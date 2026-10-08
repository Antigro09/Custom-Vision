# Consumer rejection examples

These files are malformed or semantically ineligible consumer inputs, not producer
golden fixtures. They are derived from a valid synthetic producer packet and
mutated solely to exercise rejection. Schema-valid examples still require the
reference and source/session gates in FIELDS.md.

`tests/test_protocol.py` covers negative/overflow sequence, wrong units, missing
required profile fields, invalid timestamp type, unresolved compact selection,
duplicate IDs, unavailable time and malformed geometry. A consumer must never
promote these examples to hardware-qualified observations.
