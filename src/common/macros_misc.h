/**
 * macros_misc: macros
 * Copyright (C) 2012  Rodrigo Rosa <rodrigorosa.lg gmail.com>, Matias Tailanian <matias tailanian.com>, Santiago Paternain <spaternain gmail.com>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @file   marcos_misc.h
 * @author Rodrigo Rosa <rodrigorosa.lg gmail.com>, Matias Tailanian <matias tailanian.com>, Santiago Paternain <spaternain gmail.com>
 * @date   Sun May 27 11:20:03 2012
 *
 * @brief macros
 */
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
