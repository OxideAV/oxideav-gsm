# GSM 06.07 (EN 300 968) half-rate digital test sequences — disks 1–2

The official GSM half-rate (GSM 06.20 / VSELP) conformance vectors,
13-bit uniform-PCM corpus, staged for `tests/conformance_hr_layout.rs`
(frame-layer validation now; the bit-exact decoder/encoder legs as the
half-rate implementation lands):

* `disk1/` — `SEQ01..SEQ03` encoder inputs (`.INP`) + encoder
  reference outputs (`.COD`), plus `SEQ05.INP`;
* `disk2/` — `SEQ01..SEQ05` decoder input sequences (`.DEC`) +
  decoder reference outputs (`.OUT`). Per the attachment's own
  read-me, the decoder leg is driven from `.DEC`, not from the
  encoder leg's `.COD`.

## Format (observed, confirmed against every staged file)

16-bit little-endian words. `.INP`/`.OUT`: one PCM sample per word,
160 per 20 ms frame. `.COD`: 20 words per frame — the 18 GSM 06.20
annex-A parameters in table A.1 order, then two transmit-side flag
words. `.DEC`: 22 words per frame — the 18 parameters, then four
receive-side flag words (error/DTX indications + the periodic
time-alignment flag).

## Provenance

Copied verbatim from the workspace docs staging area
(`docs/audio/gsm/conformance/halfrate-06.07/`, disks 1–2), extracted
from the ETSI EN 300 968 V8.0.1 electronic attachment
(`en_300968v080001p0.zip`); the 3GPP spec-archive copy is
byte-identical. Only data files are staged. Integrity: `SHA256SUMS`
(`shasum -a 256 -c SHA256SUMS`).

Disks 3–4 (DTX sequences) and disk 5 (synchronization sequences) stay
in the docs staging area until the corresponding half-rate features
are implemented.

ETSI distributes the deliverable and its electronic attachment as a
freely-available download; the files are reproduced here unmodified
for conformance testing.
