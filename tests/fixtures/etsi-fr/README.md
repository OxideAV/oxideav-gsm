# ETSI EN 300 961 §6 digital test sequences (13-bit uniform PCM corpus)

The official GSM full-rate conformance vectors, staged for
`tests/conformance_etsi_fr.rs`:

* `Seq01..Seq04` — `.inp` / `.cod` / `.out` (encoder + decoder legs);
  `Seq05` — `.cod` / `.out` only (decoder-only, scans all code values
  incl. out-of-range LTP lags `Nr ∈ [0,127]`).
* `Seq01h..Seq05h` — the same sequences with two codec-homing frames
  prepended (§6.2: the first output frame is undefined and need not
  match; every subsequent frame must).
* `Seq06h` — one encoder-homing frame (`.inp`) / one decoder-homing
  frame (`.cod`) (§6.3.3).
* `Homing01` — `.cod` / `.out`, the §6.3.3.2 extensive decoder-homing
  test (mixture of complete and fractional homing frames + speech).
* `Bitsync.inp`, `Seqsync.inp`, `Sync000.cod..Sync159.cod` — the
  §6.3.3.3/§6.3.3.4 bit- and frame-synchronization sequences.

## Format

16-bit **little-endian** words. `*.inp`/`*.out`: one left-justified
13-bit PCM sample per word (3 LSBs zero), 160 words per 20 ms frame.
`*.cod`: one right-justified coded parameter per word, 76 words per
frame in §1.7 Table 1.1 order. See `src/confio.rs`.

## Provenance

Copied verbatim from the workspace docs staging area
(`docs/audio/gsm/conformance/fullrate-06.10/`), which extracted the
data files of DISK1–DISK3 of the ETSI EN 300 961 V8.1.1 electronic
attachment (`en_300961v080101p0.zip`); the 3GPP spec-archive copy of
the same corpus is byte-identical. Only **data** files are staged —
the DOS executables distributed on the companion disks are not part
of this corpus. Integrity: `SHA256SUMS` (verify with
`shasum -a 256 -c SHA256SUMS`).

ETSI distributes the deliverable and its electronic attachment as a
freely-available download; the files are reproduced here unmodified
for conformance testing.

## Known defect in the staged `Sync15x` files

`Sync150.cod..Sync159.cod` are byte-identical to
`Sync050.cod..Sync059.cod`. That contradicts §6.3.3.3 ("it was
verified that all 160 output frames were different"), so these ten
files cannot be the true SYNC15x vectors. The conformance test pins
bit-exactness for `Sync000..Sync149` and pins the duplication itself
as a tripwire; when corrected vectors are staged the test will fail
loudly and should be widened back to the full range.
