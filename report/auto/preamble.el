(TeX-add-style-hook
 "preamble"
 (lambda ()
   (TeX-add-to-alist 'LaTeX-provided-package-options
                     '(("geometry" "a4paper" "top=1in" "bottom=1.1in" "left=1in" "right=1in") ("inputenc" "utf8") ("fontenc" "T1")))
   (add-to-list 'LaTeX-verbatim-environments-local "lstlisting")
   (add-to-list 'LaTeX-verbatim-macros-with-braces-local "lstinline")
   (add-to-list 'LaTeX-verbatim-macros-with-delims-local "lstinline")
   (TeX-run-style-hooks
    "graphicx"
    "caption"
    "geometry"
    "inputenc"
    "fontenc"
    "xcolor"
    "listings"
    "subcaption"
    "siunitx"
    "wrapfig"
    "varioref"
    "amsmath"
    "commath"
    "mathtools")
   (TeX-add-symbols
    '("di" 2))
   (LaTeX-add-caption-DeclareCaptions
    '("\\DeclareCaptionFormat{myformat}" "Format" "myformat"))))

