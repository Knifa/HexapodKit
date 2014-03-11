final:
	latexmk -silent -gg -dvi- -ps- -pdf -f report.tex
	make clean

preview:
	latexmk -silent -gg -dvi- -ps- -pdf -pvc -f report.tex
	make clean

clean:
	latexmk -c
	rm -f *.log *.bbl