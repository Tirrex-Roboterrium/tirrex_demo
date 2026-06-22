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
./export-pdf.sh
```

Le PDF est généré dans :

```text
public/roboterrium-slides.pdf
```

Un autre chemin de sortie peut être fourni en argument :

```bash
./export-pdf.sh /tmp/roboterrium.pdf
```

L'export utilise le mode d'impression de Reveal.js et nécessite Chrome ou
Chromium.
