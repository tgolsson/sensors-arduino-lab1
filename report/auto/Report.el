(TeX-add-style-hook
 "Report"
 (lambda ()
   (TeX-add-to-alist 'LaTeX-provided-class-options
                     '(("article" "11pt")))
   (add-to-list 'LaTeX-verbatim-environments-local "lstlisting")
   (add-to-list 'LaTeX-verbatim-macros-with-braces-local "lstinline")
   (add-to-list 'LaTeX-verbatim-macros-with-delims-local "lstinline")
   (TeX-run-style-hooks
    "latex2e"
    "preamble"
    "article"
    "art11")
   (LaTeX-add-labels
    "eq:pid"
    "eq:mje"
    "eq:tau_substitution"
    "eq:dx_dtau"
    "eq:dtau_resubs"
    "eq:dx_dt"
    "fig:pos"
    "fig:ve")))

