#!/usr/bin/env python3
"""
Generate results index for GitHub Pages viewer.

This script scans the results/ directory for JSON files and creates
an index file (results-index.json) that lists all available results.
"""

import json
import os
from pathlib import Path


def generate_results_index():
    """Scan results directory and generate index file."""
    # Define paths (script is in docs/, results is in parent directory)
    results_dir = Path(__file__).parent.parent / 'results'
    output_file = Path(__file__).parent / 'results-index.json'

    # Check if results directory exists
    if not results_dir.exists():
        print(f"Error: Results directory not found: {results_dir}")
        return

    # Find all JSON files in results directory
    json_files = sorted(
        [f.name for f in results_dir.glob('*.json')],
        reverse=True  # Most recent first (assuming alphabetical naming)
    )

    if not json_files:
        print(f"Warning: No JSON files found in {results_dir}")
        json_files = []

    # Create docs directory if it doesn't exist
    output_file.parent.mkdir(parents=True, exist_ok=True)

    # Write index file
    with output_file.open('w') as f:
        json.dump(json_files, f, indent=2)

    print(f"✓ Generated index with {len(json_files)} result(s)")
    print(f"✓ Written to: {output_file}")

    if json_files:
        print(f"\nAvailable results:")
        for filename in json_files:
            print(f"  - {filename}")


if __name__ == '__main__':
    generate_results_index()
