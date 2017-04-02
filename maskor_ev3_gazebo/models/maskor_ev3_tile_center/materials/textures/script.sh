#!/bin/bash
for f in *.png
do
   convert $f -bordercolor White -border 100x100 border_$f
   convert border_$f -flop border_$f
done
