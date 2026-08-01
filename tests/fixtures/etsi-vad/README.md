# ETSI EN 300 965 (GSM 06.32) VAD digital test sequences

The official full-rate VAD conformance corpus, staged for
`tests/conformance_vad.rs`: twenty test cases, each a triple

* `*.INP` — 13-bit uniform-PCM encoder input (16-bit LE words,
  160 words per 20 ms frame);
* `*.COD` — the coded parameters for that input (76 words per frame)
  with the VAD flag in bit 15 of the first word (LAR(1)) and the
  GSM 06.31 SP flag in bit 15 of the second word (LAR(2)); during
  DTX the frames carry the GSM 06.12 SID parameters (clause 4.1);
* `*.VAD` — ASCII, one `"<0|1> \r\n"` record per frame: the reference
  VAD flag.

Case groups (clause A.2.1): `SPEC_A*`/`SPEC_C*` (spectral-comparison
arithmetic/control), `ADAPT_I*`/`ADAPT_M*` (threshold adaptation;
run first — a broken VAD decision fails everything), `FREQ_SW`
(tone-detector frequency sweep; downlink VAD only), `PRED*`
(prediction gain), `PITCH*` (periodicity), `POLE*` (spectral pole),
`SAFETY`, `GOOD_SP`/`BAD_SP` (very clean / barely detectable speech).

## Provenance

Copied verbatim from the workspace docs staging area
(`docs/audio/gsm/conformance/vad-06.32/`), extracted from the ETSI
EN 300 965 V8.0.1 electronic attachment (`en_300965v080001p0.zip`);
the 3GPP spec-archive copy is byte-identical file-by-file. Only data
files are staged. Integrity: `SHA256SUMS`
(`shasum -a 256 -c SHA256SUMS`).

ETSI distributes the deliverable and its electronic attachment as a
freely-available download; the files are reproduced here unmodified
for conformance testing.
