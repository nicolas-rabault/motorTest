// Load results index and populate dropdown
async function loadResults() {
    const loading = document.getElementById('loading');
    const content = document.getElementById('content');
    const errorMessage = document.getElementById('error-message');
    const select = document.getElementById('result-select');
    const viewButton = document.getElementById('view-button');
    const resultCount = document.getElementById('result-count');

    try {
        // Fetch the results index
        const response = await fetch('results-index.json');

        if (!response.ok) {
            throw new Error('Failed to load results index. Make sure to run generate_index.py first.');
        }

        const results = await response.json();

        // Hide loading, show content
        loading.style.display = 'none';
        content.style.display = 'block';

        if (results.length === 0) {
            errorMessage.textContent = 'No results found. Generate some motor test results first.';
            errorMessage.style.display = 'block';
            return;
        }

        // Populate dropdown
        results.forEach(filename => {
            const option = document.createElement('option');
            option.value = filename;
            option.textContent = formatFilename(filename);
            select.appendChild(option);
        });

        // Update count
        resultCount.textContent = `${results.length} result${results.length !== 1 ? 's' : ''} available`;

        // Enable button when selection changes
        select.addEventListener('change', () => {
            viewButton.disabled = select.value === '';
        });

        // Handle view button click
        viewButton.addEventListener('click', () => {
            if (select.value) {
                // Get the base path by going up one level from docs/
                const currentPath = window.location.pathname;
                let basePath;

                if (currentPath.includes('/docs/')) {
                    // Extract everything before /docs/
                    basePath = currentPath.substring(0, currentPath.indexOf('/docs/'));
                } else if (currentPath.endsWith('/docs')) {
                    // Remove /docs from the end
                    basePath = currentPath.substring(0, currentPath.lastIndexOf('/docs'));
                } else {
                    // Fallback: use parent directory
                    basePath = '..';
                }

                // Redirect to viewer with the selected file
                window.location.href = `${basePath}/viewer.html?file=${encodeURIComponent(select.value)}`;
            }
        });

        // Handle Enter key on select
        select.addEventListener('keypress', (e) => {
            if (e.key === 'Enter' && select.value) {
                viewButton.click();
            }
        });

    } catch (error) {
        loading.style.display = 'none';
        errorMessage.textContent = error.message;
        errorMessage.style.display = 'block';
        console.error('Error loading results:', error);
    }
}

// Format filename for display
function formatFilename(filename) {
    // Remove .json extension and make more readable
    let name = filename.replace('.json', '');

    // Replace underscores and hyphens with spaces
    name = name.replace(/[_-]/g, ' ');

    // Capitalize first letter
    name = name.charAt(0).toUpperCase() + name.slice(1);

    return name;
}

// Load results when page loads
document.addEventListener('DOMContentLoaded', loadResults);
