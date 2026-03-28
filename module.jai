#import "Basic";

#import "tbx/time";
#import "tbx/circular_array";

Networked_Periodic_Signal_Quantizer :: struct(T: Type, MONITOR: bool = false) {
    received_server_states: [..] T;
    
    output_signal: Periodic_Signal;
    output_signal_monitor : Periodic_Timer;
    
    pushed_first_element: bool;
    
    // NOTE: the higher this is the more delay there will be, but the lower the probability you'll not have something
    // to grab from the buffer
    target_num_states: u32;
    max_num_states: u32;
    
    repopulate_state_buffer: bool = true;
    total_emit_opportunities: u64;
    missed_emit_opportunities: u64;

    #if MONITOR {
        MONITOR_HISTORY :: 256;
        
        // recorded every emit opportunity: how many items are in the buffer
        buffer_occupancy_history: Circular_Array(float, MONITOR_HISTORY);
        
        // recorded every emit opportunity: 1.0 = normal mode, 0.0 = repopulate mode
        mode_history: Circular_Array(float, MONITOR_HISTORY);
        
        // recorded every emit opportunity: 1.0 = emitted with value, 0.0 = emitted without value
        emit_result_history: Circular_Array(float, MONITOR_HISTORY);
        
        // recorded every push: how many items were removed due to max_num_states overflow
        removal_count_history: Circular_Array(float, MONITOR_HISTORY);
    }
}

init :: (npsq: *Networked_Periodic_Signal_Quantizer($T, $MONITOR), target_num_states: u32, max_num_states: u32, frequency_hz: float) {
    npsq.target_num_states = target_num_states;
    npsq.max_num_states = max_num_states;
    npsq.output_signal.frequency_hz = frequency_hz;
}

create :: ($T: Type, $MONITOR: bool = false, target_num_states: u32, max_num_states: u32, frequency_hz: float) -> Networked_Periodic_Signal_Quantizer(T, MONITOR) {
    npsq: Networked_Periodic_Signal_Quantizer(T, MONITOR);
    init(*npsq, target_num_states, max_num_states, frequency_hz);
    return npsq;
}

// rename to npsq add later.
push :: (using npsq: *Networked_Periodic_Signal_Quantizer($T, $MONITOR), item: T) {
    array_add(*received_server_states, item);
    
    removals := 0;
    while received_server_states.count > max_num_states {
        array_ordered_remove_by_index(*received_server_states, 0);
        removals += 1;
    }

    #if MONITOR {
        push(*removal_count_history, cast(float) removals);
    }
    
    if !pushed_first_element {
        pushed_first_element = true;
        output_signal.last_time = current_time_monotonic();
    }
}

// returns the next state if it's time to emit and one is available, success=false otherwise.
per_frame_update :: (using npsq: *Networked_Periodic_Signal_Quantizer($T, $MONITOR)) -> T, just_emitted: bool, has_value: bool {

    if !pushed_first_element {
        return .{}, false, false;
    }
    
    if ready(*output_signal) {
        mark_cycle(*output_signal_monitor);
        total_emit_opportunities += 1;
        
        #if MONITOR {
            push(*buffer_occupancy_history, cast(float) received_server_states.count);
        }

        if repopulate_state_buffer {
            if received_server_states.count >= target_num_states {
                repopulate_state_buffer = false;
            }
            missed_emit_opportunities += 1;

            #if MONITOR {
                push(*mode_history, 0.0);
                push(*emit_result_history, 0.0);
            }

            return .{}, true, false;
        } else {
            if received_server_states.count == 0 {
                repopulate_state_buffer = true;

                #if MONITOR {
                    push(*mode_history, 0.0);
                    push(*emit_result_history, 0.0);
                }

                return .{}, true, false;
            }
            
            // copy first element, then remove from front
            result := received_server_states[0];
            array_ordered_remove_by_index(*received_server_states, 0);

            #if MONITOR {
                push(*mode_history, 1.0);
                push(*emit_result_history, 1.0);
            }
            
            return result, true, true;
        }
    }
    
    return .{}, false, false;
}

get_missed_emit_percentage :: (using npsq: Networked_Periodic_Signal_Quantizer) -> float64 {
    if total_emit_opportunities == 0 {
        return 0.0;
    }
    return cast(float64) missed_emit_opportunities * 100.0 / cast(float64) total_emit_opportunities;
}

deinit :: (using npsq: *Networked_Periodic_Signal_Quantizer) {
    array_free(received_server_states);
}

// another idea is that we could wait until it's fully iniitalized or something and then not have booleans everywhere.

Networked_Interpolator :: struct(T: Type) {

    Interpolation_Window :: struct {
        start: T;
        end: T;
        t: float64;
    }
    
    // this is a pointer to a constant because functions are constant.
    Interpolation_Func :: #type (start: T, end: T, t: float) -> T;
    
    npsq: Networked_Periodic_Signal_Quantizer(T, true);
    
    interpolate_func: Interpolation_Func;

    active_start_state: T;
    active_end_state: T;
    active_t: float;
    current_interpolated_state: T;

    has_active_start_state: bool;
    has_active_end_state: bool;
    has_current_interpolated_state: bool;

    first_interpolation_window_has_been_initialized: bool;
    updated_interpolation_window_on_last_per_frame_update := false;
}

init :: (ni: *Networked_Interpolator($T), interpolate_func: Networked_Interpolator(T).Interpolation_Func, receive_frequency: float) {
    ni.interpolate_func = interpolate_func;
    init(*ni.npsq, 2, 3, receive_frequency);
    
    // reset other fields to defaults (in case we're re-initializing)
    ni.active_start_state = T.{};
    ni.active_end_state = T.{};
    ni.active_t = 0;
    ni.current_interpolated_state = T.{};
    ni.has_active_start_state = false;
    ni.has_active_end_state = false;
    ni.has_current_interpolated_state = false;
    ni.first_interpolation_window_has_been_initialized = false;
}

create :: ($T: Type, interpolate_func: Networked_Interpolator(T).Interpolation_Func, receive_frequency: float) -> Networked_Interpolator(T) {
    ni: Networked_Interpolator(T);
    init(*ni, interpolate_func, receive_frequency);
    return ni;
}

register_new_state :: (using ni: *Networked_Interpolator($T), new_state: T) {
    // log("register_new_state");
    push(*npsq, new_state);
}

get_interpolation_window :: (using ni: *Networked_Interpolator($T)) -> Networked_Interpolator(T).Interpolation_Window, bool {
    if has_active_start_state && has_active_end_state {
        window: Networked_Interpolator(T).Interpolation_Window;
        window.start = active_start_state;
        window.end = active_end_state;
        window.t = active_t;
        return window, true;
    }
    return .{}, false;
}

per_frame_update :: (using ni: *Networked_Interpolator($T)) {
    // log("nipfu start");
    // defer log("nipfu end");

    updated_interpolation_window_on_last_per_frame_update = false;

    // internally calls ready on the output_signal (1)
    // doing this is wrong, we need to know when it emits something
    // rather than just doing the per frame update all the time.
    new_state, just_emitted, has_value := per_frame_update(*npsq);

    if (just_emitted) {
        // log("npsq size: % has value: %", npsq.received_server_states.count, has_value);
        if has_value {
            // log("npsq emitted value");
            if !has_active_start_state {
                active_start_state = new_state;
                has_active_start_state = true;
                return;
            } else if !has_active_end_state {
                active_end_state = new_state;
                has_active_end_state = true;
                first_interpolation_window_has_been_initialized = true;
                return;
            }
            
            active_start_state = active_end_state;
            active_end_state = new_state;
            updated_interpolation_window_on_last_per_frame_update = true;

        } else {
             // log("npsq emitted empty");
            // no new state available - create "constant interpolation window"
            // to avoid repeating the same interpolation, instead keeps state in place.

            if has_active_end_state {
                active_start_state = active_end_state;
            }
        }
    } else {
        // log("wasn't time to emit");
    }

    
    if !first_interpolation_window_has_been_initialized {
        return;
    }
    
    // makes sense that we use this because of (1)
    active_t = npsq.output_signal.cycle_progress_at_last_ready_call;

    // if you just emitted then active t should be zero, the cycle progress at the last ready call
    // will still be 1, because that's what it was at the last ready call, but because we're
    // starting a new interpolation frame we need to force that to be 0
    if (just_emitted && active_t == 1) {
        active_t = 0;
    }
    
    current_interpolated_state = interpolate_func(active_start_state, active_end_state, cast(float) active_t);
    has_current_interpolated_state = true;
}

