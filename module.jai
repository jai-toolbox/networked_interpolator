#import "Basic";

#import "tbx/time";
#import "tbx/circular_array";

#load "networked_periodic_signal_quantizer.jai";

// another idea is that we could wait until it's fully iniitalized or something and then not have booleans everywhere.

Networked_Interpolator :: struct(T: Type) {

    Interpolation_Window :: struct {
        start: T;
        end: T;
        t: float64;
    }
    
    // this is a pointer to a constant because functions are constant.
    Interpolation_Func :: #type (start: T, end: T, t: float) -> T;
    
    npsq: Networked_Periodic_Signal_Quantizer(T);
    
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

init :: (ni: *Networked_Interpolator($T), interpolate_func: Networked_Interpolator(T).Interpolation_Func, deinit_func: Networked_Periodic_Signal_Quantizer(T).Deinit_Func, receive_frequency: float) {
    ni.interpolate_func = interpolate_func;
    init(*ni.npsq, 2, 3, 0, receive_frequency, deinit_func);
    
    ni.active_start_state = T.{};
    ni.active_end_state = T.{};
    ni.active_t = 0;
    ni.current_interpolated_state = T.{};
    ni.has_active_start_state = false;
    ni.has_active_end_state = false;
    ni.has_current_interpolated_state = false;
    ni.first_interpolation_window_has_been_initialized = false;
}

create :: ($T: Type, interpolate_func: Networked_Interpolator(T).Interpolation_Func, deinit_func: Networked_Periodic_Signal_Quantizer(T).Deinit_Func, receive_frequency: float) -> Networked_Interpolator(T) {
    ni: Networked_Interpolator(T);
    init(*ni, interpolate_func, deinit_func, receive_frequency);  
    return ni;
}

deinit :: (networked_interpolator: *Networked_Interpolator($T)) {
    npsq := *networked_interpolator.npsq;
    if networked_interpolator.has_active_start_state    maybe_deinit_item(npsq, *networked_interpolator.active_start_state);
    if networked_interpolator.has_active_end_state      maybe_deinit_item(npsq, *networked_interpolator.active_end_state);
    if networked_interpolator.has_current_interpolated_state  maybe_deinit_item(npsq, *networked_interpolator.current_interpolated_state);
    deinit(npsq);
}

create :: ($T: Type, interpolate_func: Networked_Interpolator(T).Interpolation_Func, deinit_func: Networked_Periodic_Signal_Quantizer(T).Deinit_Func, receive_frequency: float) -> Networked_Interpolator(T) {
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

    // overflow := drain_overflow(*npsq);, maybe we should do this here, but for now it happens in the other file


    // first get overflow, then we do per frame update, then we merge forward all the data.
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

