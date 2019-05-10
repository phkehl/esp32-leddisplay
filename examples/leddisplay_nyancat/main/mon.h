/*!
    \file
    \brief u-clox: system monitor (see \ref FF_MON)

    - Copyright (c) 2019 Philippe Kehl & flipflip industries (flipflip at oinkzwurgl dot org),
      https://oinkzwurgl.org/projaeggd/u-clox

    \defgroup FF_MON MON
    \ingroup FF

    @{
*/
#ifndef __MON_H__
#define __MON_H__

//! start system monitor
void monStart(void);

//! set monitor period [ms] (0 = monitor off)
void monSetPeriod(const int period);


#endif // __MON_H__
//@}
