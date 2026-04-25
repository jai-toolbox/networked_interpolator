#import "Basic";

#import "tbx/time";
#import "tbx/circular_array";

Networked_Periodic_Signal_Quantizer :: struct(T: Type, MONITOR: bool = false) {

    Deinit_Func :: #type (item: *T);

    deinit_item: Deinit_Func;  // null means no cleanup needed

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

init :: (npsq: *Networked_Periodic_Signal_Quantizer($T, $MONITOR), target_num_states: u32, max_num_states: u32, frequency_hz: float, deinit_item: Networked_Periodic_Signal_Quantizer(T, MONITOR).Deinit_Func = null) {
    npsq.target_num_states = target_num_states;
    npsq.max_num_states = max_num_states;
    npsq.output_signal.frequency_hz = frequency_hz;
    npsq.deinit_item = deinit_item;
}


create :: ($T: Type, interpolate_func: Networked_Interpolator(T).Interpolation_Func, deinit_func: Networked_Periodic_Signal_Quantizer(T, true).Deinit_Func, receive_frequency: float) -> Networked_Interpolator(T) {
    ni: Networked_Interpolator(T);
    init(*ni, interpolate_func, deinit_func, receive_frequency);  
    return ni;
}

push :: (using npsq: *Networked_Periodic_Signal_Quantizer($T, $MONITOR), item: T) {
    array_add(*received_server_states, item);

    // no overflow handling here, the caller is responsible for drain_overflow

    if !pushed_first_element {
        pushed_first_element = true;
        output_signal.last_time = current_time_monotonic();
    }
}

drain_overflow :: (using npsq: *Networked_Periodic_Signal_Quantizer($T, $MONITOR)) -> [..] T {
    removed: [..] T;
    while received_server_states.count >= max_num_states && received_server_states.count > 0 {
        array_add(*removed, received_server_states[0]);
        array_ordered_remove_by_index(*received_server_states, 0);
    }

    #if MONITOR {
        if removed.count > 0  carray_add(*removal_count_history, cast(float) removed.count);
    }

    return removed;
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
            carray_add(*buffer_occupancy_history, cast(float) received_server_states.count);
        }

        if repopulate_state_buffer {
            if received_server_states.count >= target_num_states {
                repopulate_state_buffer = false;
            }
            missed_emit_opportunities += 1;

            #if MONITOR {
                carray_add(*mode_history, 0.0);
                carray_add(*emit_result_history, 0.0);
            }

            return .{}, true, false;
        } else {
            if received_server_states.count == 0 {
                repopulate_state_buffer = true;

                #if MONITOR {
                    carray_add(*mode_history, 0.0);
                    carray_add(*emit_result_history, 0.0);
                }

                return .{}, true, false;
            }
            
            // copy first element, then remove from front
            result := received_server_states[0];
            array_ordered_remove_by_index(*received_server_states, 0);

            #if MONITOR {
                carray_add(*mode_history, 1.0);
                carray_add(*emit_result_history, 1.0);
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

maybe_deinit_item :: (using npsq: *Networked_Periodic_Signal_Quantizer($T, $MONITOR), item: *T) {
    if deinit_item  deinit_item(item);
}

deinit :: (using npsq: *Networked_Periodic_Signal_Quantizer($T, $MONITOR)) {
    for *received_server_states  maybe_deinit_item(npsq, it);
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

init :: (ni: *Networked_Interpolator($T), interpolate_func: Networked_Interpolator(T).Interpolation_Func, deinit_func: Networked_Periodic_Signal_Quantizer(T, true).Deinit_Func, receive_frequency: float) {
    ni.interpolate_func = interpolate_func;
    init(*ni.npsq, 2, 3, receive_frequency, deinit_func);
    
    ni.active_start_state = T.{};
    ni.active_end_state = T.{};
    ni.active_t = 0;
    ni.current_interpolated_state = T.{};
    ni.has_active_start_state = false;
    ni.has_active_end_state = false;
    ni.has_current_interpolated_state = false;
    ni.first_interpolation_window_has_been_initialized = false;
}

deinit :: (networked_interpolator: *Networked_Interpolator($T)) {
    npsq := *networked_interpolator.npsq;
    if networked_interpolator.has_active_start_state    maybe_deinit_item(npsq, *networked_interpolator.active_start_state);
    if networked_interpolator.has_active_end_state      maybe_deinit_item(npsq, *networked_interpolator.active_end_state);
    if networked_interpolator.has_current_interpolated_state  maybe_deinit_item(npsq, *networked_interpolator.current_interpolated_state);
    deinit(npsq);
}

create :: ($T: Type, interpolate_func: Networked_Interpolator(T).Interpolation_Func, deinit_func: Networked_Periodic_Signal_Quantizer(T, true).Deinit_Func, receive_frequency: float) -> Networked_Interpolator(T) {
    ni: Networked_Interpolator(T);
    init(*ni, interpolate_func, receive_frequency, deinit_func);
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
    updated_interpolation_window_on_last_per_frame_update = false;

    new_state, just_emitted, has_value := per_frame_update(*npsq);

    if (just_emitted) {
        if has_value {
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
            
            maybe_deinit_item(*npsq, *active_start_state);
            active_start_state = active_end_state;
            active_end_state = new_state;
            updated_interpolation_window_on_last_per_frame_update = true;

        } else {
            if has_active_end_state {
                maybe_deinit_item(*npsq, *active_start_state);
                active_start_state = active_end_state;
                active_end_state = T.{};
                has_active_end_state = false;
                first_interpolation_window_has_been_initialized = false;
            }
        }
    }

    if !first_interpolation_window_has_been_initialized {
        return;
    }
    
    active_t = npsq.output_signal.cycle_progress_at_last_ready_call;
    if (just_emitted && active_t == 1) {
        active_t = 0;
    }
    
    if has_current_interpolated_state {
        maybe_deinit_item(*npsq, *current_interpolated_state);
    }
    current_interpolated_state = interpolate_func(active_start_state, active_end_state, cast(float) active_t);
    has_current_interpolated_state = true;
}

