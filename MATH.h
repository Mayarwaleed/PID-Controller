#ifndef MATH_H_
#define MATH_H_

#define SET_BIT(REG,BIT)     (REG |= (1<<BIT))
#define CLR_BIT(REG,BIT)     (REG &= ~(1<<BIT))
#define TOOGLE_BIT(REG,BIT)  (REG ^= 1<<BIT)
#define GET_BIT(REG,BIT)     ((REG>>BIT) & 1)

#endif