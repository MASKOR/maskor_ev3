# ############################################################################
#
#            $Id: Makefile 7 2015-01-27 17:50:03Z stf $
#         author: Stefan Schiffer
#    description: standard LaTeX-Makefile
#  last modified: $Date: 2015-01-27 18:50:03 +0100 (Tue, 27 Jan 2015) $    
#             by: $Author: stf $
#
# ############################################################################

#SRC  = 	figure-a
#SRC  = 	figure-a_lsg
##SRC  = 	figure-b
#SRC  = 	figure-b-i_lsg
#SRC  = 	figure-b-ii_lsg

TILDEFILES = *~

## COLORS #################

SILENT = @
LATEX = latex
BIBTEX = bibtex
DVIPS = dvips
DVIPSFLAGS = -Ppdf -G0
DVIPDF = dvipdf
PS2PDF = ps2pdf
PS2PDFFLAGS =  -sPAPERSIZE=a4 -dCompatibilityLevel=1.3 -dEmbedAllFonts=true
PSNUP = psnup
PDFLATEX = pdflatex
VERSION = version.txt
TAR = tar -czf
UNTAR = tar -xzf
TARFILE = $(PREFIX).tgz
RM = rm -f

## COLORS #################

MAKE = make 
DBG  = echo
MSG  = echo

## COLORS #################

RESET       = tput sgr0
#
BLACK       = tput setaf 0
BLACK_BG    = tput setab 0
DARKGREY    = tput setaf 0; tput bold
RED         = tput setaf 1
RED_BG      = tput setab 1
LIGHTRED    = tput setaf 1; tput bold
GREEN       = tput setaf 2
GREEN_BG    = tput setab 2
LIME        = tput setaf 2; tput bold
BROWN       = tput setaf 3
BROWN_BG    = tput setab 3
YELLOW      = tput setaf 3; tput bold
BLUE        = tput setaf 4
BLUE_BG     = tput setab 4
BRIGHTBLUE  = tput setaf 4; tput bold
PURPLE      = tput setaf 5
PURPLE_BG   = tput setab 5
PINK        = tput setaf 5; tput bold
CYAN        = tput setaf 6
CYAN_BG     = tput setab 6
BRIGHTCYAN  = tput setaf 6; tput bold
LIGHTGREY   = tput setaf 7
LIGHTGREYBG = tput setab 7
WHITE       = tput setaf 7; tput bold
#
# AND NOW FOR NAMED-HELPER:
MENU  = $(CYAN)
ITEM  = $(BRIGHTCYAN)
DONE  = $(CYAN)
CHECK = $(GREEN)
ERROR = $(RED)



## #################################### ##
##  R U L E S                           ##
## #################################### ##

#TEX = $(SRC:%=%.tex)
#DVI = $(TEX:%.tex=%.dvi)
#PS = $(DVI:%.dvi=%.ps)
#PDF = $(PS:%.ps=%.pdf)

PDF = $(TEX:%.tex=%.pdf)
PNG = $(PDF:%.pdf=%.png)
JPG = $(PNG:%.png=%.jpg)

#.PHONY: 
#	all %.pdf $(PDF) 

all:	scenario


scenario:
	$(SILENT) $(MENU); $(MSG) " -- BUILDING ALL PICTURES ----------------------------------------------"; $(RESET)
	$(SILENT) make -s scenario-layout-hash.pdf
	$(SILENT) $(DONE); $(MSG) " --- DONE --------------------------------------------------------------"; $(RESET)

#	$(SILENT) pdflatex scenario-layout-hash.tex

calibration:
	$(SILENT) $(MENU); $(MSG) " -- BUILDING ALL PICTURES ----------------------------------------------"; $(RESET)
	$(SILENT) make -s scenario-calibration.pdf
	$(SILENT) $(DONE); $(MSG) " --- DONE --------------------------------------------------------------"; $(RESET)

pdf:
	$(SILENT) $(MENU); $(MSG) " -- BUILDING ALL PDFs --------------------------------------------------"; $(RESET)
	$(SILENT) make -s scenario-layout-hash.pdf
	$(SILENT) make -s scenario-calibration.pdf
	$(SILENT) $(DONE); $(MSG) " --- DONE --------------------------------------------------------------"; $(RESET)


#	$(SILENT) make *.png

png:
	$(SILENT) $(MENU); $(MSG) " -- BUILDING ALL PNGs --------------------------------------------------"; $(RESET)
	$(SILENT) make -s figure-a.png
	$(SILENT) make -s figure-b-i.png
	$(SILENT) $(DONE); $(MSG) " --- DONE --------------------------------------------------------------"; $(RESET)


mall:	$(PDF) $(PNG)
	$(SILENT) $(DONE); $(MSG) " --- DONE --------------------------------------------------------------"; $(RESET)


%.pdf: %.tex 
	$(SILENT) $(MENU); $(MSG) " -----------------------------------------------------------------------"; $(RESET)
	$(SILENT) $(ITEM); $(MSG) " --- Creating" $@ "via $(PDFLATEX)"; $(RESET)
	$(SILENT) $(PDFLATEX) $<
	$(SILENT) $(ITEM); $(MSG) " --- 2nd run $(PDFLATEX)" $@ ; $(RESET)
	$(SILENT) $(PDFLATEX) $<
	$(SILENT) $(DONE); $(MSG) " ------------------------------------------------------------- done. ---"; $(RESET)

#optimize:
#	$(SILENT) $(MSG) "  -- Running GS to optimize '$(SRC).pdf'"
#	gs -q -dOptimize=true -dNOPAUSE -dSAFER -sDEVICE=pdfwrite -sOutputFile=$(SRC).opt.pdf $(SRC).pdf -c quit

%.png: %.pdf
	$(SILENT) $(MENU); $(MSG) " -----------------------------------------------------------------------"; $(RESET)
	$(SILENT) $(MSG) "  -- Running GS to generate png from '$<'"
	$(SILENT) gs -dSAFER -dNOPAUSE -r400 -dGraphicsAlphaBits=4 -dTextAlphaBits=4 -sDEVICE=png16m -sOutputFile=$@ -dBATCH "$<"
	$(SILENT) $(DONE); $(MSG) " ------------------------------------------------------------- done. ---"; $(RESET)

%.jpg: %.png
	$(SILENT) $(MENU); $(MSG) " -----------------------------------------------------------------------"; $(RESET)
	$(SILENT) $(MSG) "  -- Converting '$<' (png) to '$@' (jpg)"
	$(SILENT) convert $< $@
	$(SILENT) $(DONE); $(MSG) " ------------------------------------------------------------- done. ---"; $(RESET)

#	$(SILENT) gs -dNOPAUSE -r400 -dGraphicsAlphaBits=4 -dTextAlphaBits=4 -sDEVICE=png16m -sOutputFile=$(SRC).png -dBATCH $(SRC).pdf

clean:
	$(SILENT) $(MENU); $(MSG) " -----------------------------------------------------------------------"; $(RESET)
	$(SILENT) $(ITEM); $(MSG) " --- Cleaning Up ..."; $(RESET)
	$(SILENT) $(RM) $(DVI) $(PS) $(PDF) *.aux *.log *.bbl *.blg $(TILDEFILES) $(TARFILE) *.snm *.out *.nav *.toc *.vrb 
	$(SILENT) $(DONE); $(MSG) " ------------------------------------------------------------- done. ---"; $(RESET)
