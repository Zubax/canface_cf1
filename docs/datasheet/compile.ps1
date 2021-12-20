#!/usr/bin/env pwsh
param ($texfile='Zubax_Babel_Datasheet.tex')
Set-Location (Split-Path $script:MyInvocation.MyCommand.Path)
pdflatex --halt-on-error --shell-escape $texfile
pdflatex --halt-on-error --shell-escape $texfile
pdflatex --halt-on-error --shell-escape $texfile
