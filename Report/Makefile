final:
	latexmk -gg -dvi- -ps- -pdf report.tex
	make clean

preview:
	latexmk -gg -dvi- -ps- -pdf -pvc report.tex
	make clean

clean:
	latexmk -c
	rm -f *.log *.bbl