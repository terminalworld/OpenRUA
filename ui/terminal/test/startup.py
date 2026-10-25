"""Run the product entry with deterministic update offers for terminal tests."""

from openrua.cli import main, updates

updates.offers = lambda composed, home: []

if __name__ == '__main__':
    raise SystemExit(main())
