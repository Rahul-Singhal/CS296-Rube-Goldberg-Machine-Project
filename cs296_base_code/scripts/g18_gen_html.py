#!/usr/bin/env python3
import 	subprocess
import sys
import re
import os

texFile = open("doc/g18_prof_report.tex" , 'r')

text = texFile.read()
text = re.sub(r'\\section{Profiling}.*end{document}' , r'' , text , flags=re.DOTALL)
text = re.sub(r'\\\\',r'<br/><br/>',text)
text = re.sub(r'\\(_|%|&)',r'\1',text)
text = re.sub(r'\\begin{center}|\\maketitle|\\end{center}|\\begin{document}|\\end{document}|\\documentclass[^}]*}|\\usepackage[^}]*}',r'',text)
text = re.sub(r'\\begin{itemize}',r'<ul>',text)
text = re.sub(r'\\end{itemize}',r'</ul>',text)
text = re.sub(r'\\item(.*)\n',r'<li>\1</li>\n',text)
text = re.sub(r'\\includegraphics[^{]*{(.*)}' , r'<div class="GRAPH"><img src = "\1"/></div>' , text , flags=re.MULTILINE)
text = re.sub(r'\\([a-zA-z]*){(.*)}' , r'<div class="\1">\2</div>' , text)
text = re.sub(r'\\author{([^}]*)}' , r'<div class="author">\1</div>' , text , flags=re.MULTILINE)
text = re.sub(r'class="date"' , r'class="date" id = "date"' , text)
text = re.sub(r'.eps' , r'.jpg' , text)
text = re.sub(r'images/g18_plot01_01.jpg' , r'plots/g18_lab09_plot01.png' , text)
text = re.sub(r'images/g18_plot01_02.jpg' , r'plots/g18_lab09_plot02.png' , text)
text = re.sub(r'images/g18_plot01_03.jpg' , r'plots/g18_lab09_plot03.png' , text)
text = re.sub(r'images/g18_plot01_04.jpg' , r'plots/g18_lab09_plot04.png' , text)
text = re.sub(r'images/g18_plot02.jpg' , r'plots/g18_lab09_plot05.png' , text)
text = re.sub(r'images/g18_plot03.jpg' , r'plots/g18_lab09_plot06.png' , text)

htmlfile = open("doc/g18_report.html" , "w+")
htmlfile.write("<html><head><script type=\"text/javascript\">		function date(){	var today = new Date();	document.getElementById(\"date\").innerHTML = \"DATE : \" + today.toLocaleString();		} 	</script><link href=\"g18_report.css\" rel=\"stylesheet\" type=\"text/css\" />	</head><body onload = \"date()\">")
htmlfile.write(text)
htmlfile.write("</body></html>")
	
	
