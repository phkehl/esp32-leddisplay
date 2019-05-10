#!/bin/bash

#Simple and stupid script to (re)generate image data. Needs an Unix-ish environment with
#ImageMagick and xxd installed.

convert nyan_64x32.gif nyan_64x32-f%02d.rgb

OUTF="nyan_64x32.c"

echo '// Auto-generated' > $OUTF
echo '#include <sdkconfig.h>' >> $OUTF
echo '#include "nyan_64x32.h"' >> $OUTF
echo 'static const unsigned char myanim[]={' >> $OUTF
{
	for x in nyan_64x32-f*.rgb; do
		echo $x >&2
		cat $x
	done

} | xxd -i >> $OUTF
echo "};" >> $OUTF

echo 'const uint8_t *get_nyan_64x32(int *nFrames) { *nFrames = 12; return myanim; }' >> $OUTF
rm -f *.rgb
