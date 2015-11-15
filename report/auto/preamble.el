(TeX-add-style-hook
 "preamble"
 (lambda ()
   (TeX-add-to-alist 'LaTeX-provided-package-options
                     '(("geometry" "a4paper" "top=1in" "bottom=1.1in" "left=1in" "right=1in") ("inputenc" "utf8") ("fontenc" "T1")))
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
    "amsmath")))

