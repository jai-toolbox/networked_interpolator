#import "Basic";

#import "tbx/circular_array";
#import "tbx/time";
#import "tbx/math";

/*

-----------
Overview

This is a thing that takes in a stream of periodic data (that has variance), and turns it into a new stream of periodic data with less variance by introducing a delay.

In many places this is called a Jitter Buffer, but ours is not as general purpose as that.

-----------
Explanation

Suppose that the input stream sends at X Hz, and has variance V which is much smaller than X (if V was close to X, then the input stream woudln't even be sending at approx X HZ).
The NPSQ's goal is to re-emit that same input stream at X Hz, with variance S where S is much smaller than V but with a delay.

If we simply try to emit the incoming input stream at X Hz, then if an input comes in late (due to V), and it's time for the NPSQ to emit then there's nothing to emit.

One way we can correct for this is to buffer N states from the input stream, that means that if its time for NPSQ to emit, and we have not received a new input,
then we can just pull from the buffer. In most cases N can be small (1, 2, 3) because usually the variance isn't that bad.

At the same time if there is a period where the input stream goes blank, then its still possible for the input buffer to become empty as we consume from it, 
if that occurs then we will still have the same issue as before, but the probably of this occuring is greatly reduced.

If the incoming buffer ever does become empty, then the NPSQ, will take some time to rebuild that buffer up to N states, before resuming regular operation.

-----------
Terminology

Input: The data that comes in from the periodic input stream
Emit: When the periodic signal controlling the NSPQ is ready then we say that it emits an input.
Overflow: Suppose we want to keep the input buffer holding 

-----------
Gotchas

The input stream, and the output stream may emit at the same rate, but they will not be synchronized, there will be a phase offset. This is also due to the fact that there will be drift because of the variance and clock differences (ie the phase offset is changing as well).

If you want to hold 2 states in the input buffer, then that means that, then the amount of latency that introduces is an expected value of 2.5 * 1/X, so it's not just 2 * 1/X, which you might initially believe.

We could have limited the size of the input buffer, using a ring buffer, but we don't do that. Instead we provide a function called drain_overflow. This allows the user to make a choice about what they want to do with overflow rather than just just removing it.

The oldest elements in the input buffer are less important than the newer ones because of delay, the overflow is always pulling off the older elements.

-----------
Usage

init: call before using
deinit: call when done 
push: put an input into the input_buffer
per_frame_update: call every frame
drain_overflow: call every frame

*/


NPSQ_Mode :: enum u8 {
    REGULAR_OPERATION;
    POPULATE_INPUT_BUFFER;
}

NPSQ_Emit_Type :: enum u8 {
    NOT_READY;
    VALUE;
    EMPTY;
}

// we could call this a Periodic_Signal_Variance_Reducer (but for now this is only used in a network context so we're just leaving the naming be)
Networked_Periodic_Signal_Quantizer :: struct(T: Type) {

    Mode :: enum u8 { REGULAR_OPERATION; POPULATE_INPUT_BUFFER; }

    // this is not something that the programmer has to set.
    mode : NPSQ_Mode = .POPULATE_INPUT_BUFFER; // at the start we need to to populate.


    // elements with small indices are older than those with high ones. 
    input_buffer: [..] T;

    // NOTE: the higher this is the more delay there will be, but the lower the probability you'll not have something
    // to grab from the buffer
    target_buffer_size : u32; // we repopulate back up to this amount.
    buffer_upper_bound : u32; // anything above this becomes overflow
    buffer_size_to_enter_populate_mode : u32; // if the input buffer becomes this value, the mode changes to POPULATE_BUFFER 


    output_signal: Periodic_Signal;
    output_signal_monitor : Periodic_Timer;

    push_monitor_timer : Periodic_Timer;
    
    pushed_first_element: bool;


    // probably don't need this anymore.
    Deinit_Func :: #type (item: *T);
    deinit_item: Deinit_Func;  // null means no cleanup needed

    HISTORY_SIZE : s32 : 60;

    // data is registered into the history everytime per-frame update is called.
    input_buffer_size_history : Circular_Array(u32, HISTORY_SIZE);
    overflow_history : Circular_Array(u32, HISTORY_SIZE);
    mode_history : Circular_Array(NPSQ_Mode, HISTORY_SIZE);

    emission_history : Circular_Array(NPSQ_Emit_Type, HISTORY_SIZE);
}

init :: (npsq: *Networked_Periodic_Signal_Quantizer($T), target_buffer_size: u32, buffer_upper_bound: u32, buffer_size_to_enter_populate_mode : u32, frequency_hz: float, deinit_item: Networked_Periodic_Signal_Quantizer(T).Deinit_Func = null) {
    npsq.target_buffer_size = target_buffer_size;
    npsq.buffer_upper_bound = buffer_upper_bound;
    npsq.buffer_size_to_enter_populate_mode = buffer_size_to_enter_populate_mode;
    npsq.output_signal.frequency_hz = frequency_hz;
    npsq.deinit_item = deinit_item;
}



push :: (using npsq: *Networked_Periodic_Signal_Quantizer($T), item: T) {
    array_add(*input_buffer, item);
    mark_cycle(*push_monitor_timer);

    // this could be in per_frame_update, but it logically makes more sense here.
    if mode == .POPULATE_INPUT_BUFFER {
        if input_buffer.count >= target_buffer_size {
            mode = .REGULAR_OPERATION;
        }
    }

    // no overflow handling here, the caller is responsible for drain_overflow

    // I think that we don't need to do this anymore?
    if !pushed_first_element {
        pushed_first_element = true;
        // why do we manually do this?
        output_signal.last_time = current_time_monotonic();
    }
}

// must be called along with per_frame_update to keep the received server states low
// the returns overflow is in oldest->newest
drain_overflow :: (using npsq: *Networked_Periodic_Signal_Quantizer($T)) -> overflow : [..] T {
    removed: [..] T;
    while input_buffer.count > buffer_upper_bound && input_buffer.count > 0 { // maybe just do an assert on the bound before hand
        array_add(*removed, input_buffer[0]);
        array_ordered_remove_by_index(*input_buffer, 0);
    }

    return removed;
}

// returns the next state if it's time to emit and one is available, success=false otherwise.
per_frame_update :: (using npsq: *Networked_Periodic_Signal_Quantizer($T)) -> T, just_emitted: bool, has_value: bool {

    carray_add(*overflow_history, saturating_sub(xx input_buffer.count, buffer_upper_bound));
    carray_add(*input_buffer_size_history, xx input_buffer.count);
    carray_add(*mode_history, mode);

    if mode == .POPULATE_INPUT_BUFFER {
        carray_add(*emission_history, .NOT_READY);
        return .{}, false, false;
    }

    assert(mode == .REGULAR_OPERATION);

    if ready(*output_signal) {
        // this is probably redundant, because we're just checking if periodic signal code works as expected
        mark_cycle(*output_signal_monitor); 

        // it's important that we do this on at the given frequency, because of the phase offset of the the two signals, if we checked this outside of when the output signal was ready we'd enter the POPULATE more frequently then we have to, because maybe in the time from now until the signal is ready push is called and we wouldn't have to enter the POPULATE mode.
        if input_buffer.count <= buffer_size_to_enter_populate_mode { // I think this can be ==, but wasn't 100%
            mode = .POPULATE_INPUT_BUFFER;
            carray_add(*emission_history, .EMPTY);
            return .{}, true, false;
        }

        assert(input_buffer.count > buffer_size_to_enter_populate_mode);

        result := input_buffer[0];
        array_ordered_remove_by_index(*input_buffer, 0);

        carray_add(*emission_history, .VALUE);
        return result, true, true;
    }


    // this is the most common return, when the output signal is not ready (because you call this function at a faster rate)
    carray_add(*emission_history, .NOT_READY);
    return .{}, false, false;
}

// log this out every second or something to understand its behavior
log_state :: (using npsq: *Networked_Periodic_Signal_Quantizer($T)) {
    log("=== NPSQ State ===");
    log("  mode:                              %", mode);
    log("  input_buffer.count:                %", input_buffer.count);
    log("  target_buffer_size:                %", target_buffer_size);
    log("  buffer_upper_bound:                %", buffer_upper_bound);
    log("  buffer_size_to_enter_populate:     %", buffer_size_to_enter_populate_mode);
    log("  output_signal.frequency_hz:        %", output_signal.frequency_hz);
    log("  pushed_first_element:              %", pushed_first_element);

    // measured rates (derived from average delta time)
    push_dt   := get_average_delta_time(*push_monitor_timer);
    output_dt := get_average_delta_time(*output_signal_monitor);
    push_hz   := ifx push_dt   > 0 then 1.0 / push_dt   else 0.0;
    output_hz := ifx output_dt > 0 then 1.0 / output_dt else 0.0;
    log("  measured push rate (Hz):           %", push_hz);
    log("  measured emit rate (Hz):           %", output_hz);

    // recent emission history
    recent_emissions_to_show :: 16;
    start_idx := max(0, emission_history.count - recent_emissions_to_show);

    builder: String_Builder;
    defer free_buffers(*builder);
    print_to_builder(*builder, "[");
    for i: start_idx..emission_history.count - 1 {
        emit := emission_history.data[i];
        if      emit == .NOT_READY  print_to_builder(*builder, ".");
        else if emit == .VALUE      print_to_builder(*builder, "V");
        else if emit == .EMPTY      print_to_builder(*builder, "E");
        if i != emission_history.count - 1 print_to_builder(*builder, " ");
    }
    print_to_builder(*builder, "]");
    log("  recent emissions (newest last):    %", builder_to_string(*builder,, allocator = temp));

    // emission totals across full history
    not_ready_count := 0;
    value_count     := 0;
    empty_count     := 0;
    for i: 0..emission_history.count - 1 {
        emit := emission_history.data[i];
        if      emit == .NOT_READY  not_ready_count += 1;
        else if emit == .VALUE      value_count     += 1;
        else if emit == .EMPTY      empty_count     += 1;
    }
    log("  emission totals (last % entries):  NOT_READY=%, VALUE=%, EMPTY=%",
        emission_history.count, not_ready_count, value_count, empty_count);

    // max overflow seen in recent history
    max_recent_overflow : u32 = 0;
    for i: 0..overflow_history.count - 1 {
        v := overflow_history.data[i];
        if v > max_recent_overflow  max_recent_overflow = v;
    }
    log("  max overflow over last %:          %", overflow_history.count, max_recent_overflow);

    log("==================");
}


maybe_deinit_item :: (using npsq: *Networked_Periodic_Signal_Quantizer($T), item: *T) {
    if deinit_item  deinit_item(item);
}

deinit :: (using npsq: *Networked_Periodic_Signal_Quantizer($T)) {
    for *input_buffer  maybe_deinit_item(npsq, it);
    array_free(input_buffer);
}
