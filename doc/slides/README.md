# Présentation INRAE / Roboterrium avec Quarto Reveal.js

Cette archive contient une présentation Quarto au style INRAE, construite à partir du template de David Carayon.

## Fichiers principaux

- `index.qmd` : présentation Roboterrium.
- `custom.scss` : styles additionnels.
- `img/` : logos INRAE et images Roboterrium utilisées dans les diapositives.
- `LANCER.md` : commandes pour prévisualiser et générer la présentation.
- `render.sh` : raccourci pour lancer `quarto preview index.qmd`.
- `export-pdf.sh` : génère le PDF avec Chrome/Chromium.

## Installation rapide

```bash
quarto add quarto-ext/fontawesome
quarto install extension davidcarayon/quarto-inrae-extension
quarto preview index.qmd
```

## Export PDF

```bash
./export-pdf.sh
```
