# PDF Generation and Troubleshooting

## Generate the Complete PDF

```bash
cd src/tirrex/tirrex_demo
MERMAID_FILTER_FORMAT=pdf pandoc \
  doc/getting_started.md \
  doc/getting_started/01_overview.md \
  doc/getting_started/02_robot.md \
  doc/getting_started/03_geographic_anchor.md \
  doc/getting_started/04_localisation.md \
  doc/getting_started/05_path_following.md \
  doc/getting_started/06_teleoperation.md \
  doc/getting_started/07_simulation.md \
  doc/getting_started/08_recording_replay.md \
  doc/getting_started/09_adap2e_walkthrough.md \
  doc/getting_started/10_add_new_component.md \
  doc/getting_started/11_create_new_demo.md \
  doc/getting_started/12_pdf_generation.md \
  -F mermaid-filter \
  -o doc/getting_started.pdf \
  --toc \
  --number-sections \
  --top-level-division=chapter \
  -V documentclass=report \
  -V geometry:margin=2cm \
  --resource-path=doc/getting_started
```

Keep the document PDF-friendly:

* use simple Markdown tables;
* keep code examples in fenced blocks;
* use Mermaid for runtime flows and architecture diagrams;
* keep directory layouts and copyable configuration structures in fenced text blocks;
* store future getting-started figures in `doc/getting_started/`;
* use relative links when possible.
* use `-V geometry:margin=2cm` to keep enough horizontal space for tables and
  code blocks.
* use `--top-level-division=chapter` with `-V documentclass=report` so each
  top-level section starts on a new page in the PDF.
* use `MERMAID_FILTER_FORMAT=pdf` so Mermaid diagrams are rendered as vector
  graphics instead of low-resolution PNG images.
* install `mermaid-filter` before generating the PDF:

```bash
npm install --global mermaid-filter
```

## Markdown Diagrams and LaTeX

Mermaid diagrams remain readable in GitHub and GitLab, and `mermaid-filter`
renders them into images before Pandoc builds the PDF.

LaTeX snippets can be used later for PDF-only figures when a diagram needs a
more polished layout, but they should not replace the basic configuration-tree
examples. Those examples are part of the user workflow and must stay easy to
copy, review and update.

## Troubleshooting

### No joystick found

Check `config/robot/devices.yaml`. For the selected mode, there must be exactly
one device with `type: joystick`.

### Device file not found

If `devices.yaml` contains:

```yaml
septentrio:
  type: gps
  available_mode: all
```

then the matching file must be:

```text
config/robot/devices/septentrio.gps.yaml
```

### Simulator world not found

Check `config/simulation.yaml`. The selected simulator must have a
`world_package` and a `world_name`.

### Wrong topics recorded

Check the `records` section of the base and device meta-descriptions. If a
device publishes a topic that does not follow the default prefix convention, add
or update its `bridge` section in the device meta-description.

### Path file not found

If the `path` launch argument is relative, `tirrex_core` resolves it under:

```text
config/paths/
```

Use either an absolute path or a file name present in this directory.
