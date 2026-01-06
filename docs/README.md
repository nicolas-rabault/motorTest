# Motor Test Results Viewer - GitHub Pages

This directory contains the GitHub Pages site for viewing motor test results online.

## Files

- **index.html** - Landing page with dropdown selector for all available results
- **app.js** - JavaScript for loading results index and handling navigation
- **results-index.json** - Auto-generated list of available result files (do not edit manually)

## How It Works

1. Users visit the landing page at `https://<username>.github.io/motorTest/`
2. The page loads `results-index.json` to populate a dropdown menu
3. When a result is selected, the user is redirected to `../viewer.html?file=<filename>`
4. The viewer loads the result from `../results/<filename>` and displays it

## Updating Results

After adding new result files to the `/results` directory, regenerate the index:

```bash
# From repository root
python3 generate_index.py
git add docs/results-index.json
git commit -m "Update results index"
git push
```

GitHub Pages will automatically rebuild and deploy within ~1 minute.

## Local Testing

To test locally before deploying:

1. Start a local server in the repository root:
   ```bash
   python3 -m http.server 8000
   ```

2. Open in browser:
   ```
   http://localhost:8000/docs/
   ```

## Structure

```
/
├── docs/                    # GitHub Pages root
│   ├── index.html          # Landing page
│   ├── app.js             # Dynamic loader
│   └── results-index.json # Auto-generated
├── viewer.html            # Result viewer (at repo root)
├── results/               # Result files (at repo root)
└── generate_index.py      # Index generator script
```

## Notes

- The viewer is kept at repository root to avoid duplication
- Results directory is kept at repository root for direct access
- Only the `/docs` folder is served as the GitHub Pages root
- Files outside `/docs` are accessible via relative paths (`../`)
