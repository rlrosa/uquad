#ifndef MACROS_MISC_H
#define MACROS_MISC_H

#define wait_for_enter while(fread(tmp,1,1,stdin) == 0)

#warning "Using crappy macro por sp_t!"
#define sp_t int

#endif //MACROS_MISC_H
