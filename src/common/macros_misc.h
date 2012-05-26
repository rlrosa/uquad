#ifndef MACROS_MISC_H
#define MACROS_MISC_H

#define wait_for_enter while(fread(tmp,1,1,stdin) == 0)

/**
 * Get previous index of circ buffer
 *
 * @param curr_index current element
 * @param buff_len length of the buffer
 *
 * @return answer
 */
#define circ_buff_prev_index(curr_index, buff_len)	\
    (--curr_index < 0)?					\
    curr_index + buff_len:				\
    curr_index;


#endif //MACROS_MISC_H
