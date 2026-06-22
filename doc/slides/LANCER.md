# Lancer la présentation Roboterrium

## 1. Prérequis

Installer Quarto : https://quarto.org/docs/get-started/

Puis installer les extensions nécessaires au template INRAE :

```bash
quarto add quarto-ext/fontawesome
quarto install extension davidcarayon/quarto-inrae-extension
```

## 2. Prévisualiser

Depuis ce dossier :

```bash
quarto preview index.qmd
```

## 3. Générer le HTML

```bash
quarto render index.qmd
```

Le fichier généré sera dans le dossier `public/`.

## 4. Exporter en PDF

```bash
quarto render index.qmd --to inrae-revealjs-pdf
```

Selon votre installation, l’export PDF peut nécessiter Chromium/Chrome.
