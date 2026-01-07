// Load comparison table with motor data
async function loadComparisonTable(resultFiles) {
    const tableContainer = document.getElementById('comparison-table-container');
    const tbody = document.getElementById('comparison-tbody');

    try {
        // Fetch all result files
        const motorDataPromises = resultFiles.map(async filename => {
            try {
                const response = await fetch(`results/${filename}`);
                if (!response.ok) return null;
                const data = await response.json();
                return { filename, data };
            } catch (error) {
                console.error(`Error loading ${filename}:`, error);
                return null;
            }
        });

        const motorDataArray = await Promise.all(motorDataPromises);
        const validMotorData = motorDataArray.filter(item => item !== null);

        if (validMotorData.length === 0) {
            return;
        }

        // Generate table rows
        tbody.innerHTML = '';
        validMotorData.forEach(({ filename, data }) => {
            const row = createComparisonRow(filename, data);
            tbody.appendChild(row);
        });

        // Show the table
        tableContainer.style.display = 'block';

    } catch (error) {
        console.error('Error loading comparison table:', error);
    }
}

// Create a comparison table row for a motor
function createComparisonRow(filename, data) {
    const row = document.createElement('tr');
    row.style.borderBottom = '1px solid #e0e0e0';
    row.style.cursor = 'pointer';
    row.style.transition = 'background-color 0.2s';

    // Add hover effect
    row.addEventListener('mouseenter', () => {
        row.style.backgroundColor = '#f5f5f5';
    });
    row.addEventListener('mouseleave', () => {
        row.style.backgroundColor = 'white';
    });

    // Make row clickable to view details
    row.addEventListener('click', () => {
        window.location.href = `viewer.html?file=${encodeURIComponent(filename)}`;
    });

    // Calculate burst torque to weight ratio
    let burstTorqueToWeight = 'N/A';
    if (data.torque_burst_test && data.torque_burst_test.data &&
        data.torque_burst_test.data.length > 0 && data.weight_g && data.weight_g > 0) {

        const burst = data.torque_burst_test;
        const steadyStateData = burst.data.filter(d =>
            d.time_relative_ms >= 10 &&
            d.time_relative_ms <= burst.duration_ms - 10
        );

        const measuredTorques = steadyStateData.map(d => d.measured_torque_Nm).filter(t => !isNaN(t));
        if (measuredTorques.length > 0) {
            const avgMeasuredTorque = measuredTorques.reduce((a, b) => a + b, 0) / measuredTorques.length;
            burstTorqueToWeight = ((avgMeasuredTorque * 1000) / data.weight_g).toFixed(3);
        }
    }

    // Calculate burst torque error
    let burstTorqueError = 'N/A';
    if (data.torque_burst_test && data.torque_burst_test.data &&
        data.torque_burst_test.data.length > 0) {

        const burst = data.torque_burst_test;
        const steadyStateData = burst.data.filter(d =>
            d.time_relative_ms >= 10 &&
            d.time_relative_ms <= burst.duration_ms - 10
        );

        const measuredTorques = steadyStateData.map(d => d.measured_torque_Nm).filter(t => !isNaN(t));
        const estimatedTorques = steadyStateData.map(d => d.estimated_torque_Nm).filter(t => !isNaN(t));

        if (measuredTorques.length > 0 && estimatedTorques.length > 0) {
            const avgMeasured = measuredTorques.reduce((a, b) => a + b, 0) / measuredTorques.length;
            const avgEstimated = estimatedTorques.reduce((a, b) => a + b, 0) / estimatedTorques.length;

            if (avgMeasured !== 0) {
                burstTorqueError = (Math.abs((avgMeasured - avgEstimated) / avgMeasured) * 100).toFixed(1);
            }
        }
    }

    // Get KT measured
    const ktMeasured = data.electrical && data.electrical.KT_Nm_per_A ?
        (data.electrical.KT_Nm_per_A * 1000).toFixed(2) : 'N/A';

    row.innerHTML = `
        <td style="padding: 12px; font-weight: 500;">${data.name || 'Unknown'}</td>
        <td style="padding: 12px; text-align: center;">
            ${data.image_url ?
                `<img src="${data.image_url}" alt="${data.name}" style="width: 60px; height: 60px; object-fit: cover; border-radius: 4px;">` :
                '<span style="color: #999;">No image</span>'}
        </td>
        <td style="padding: 12px; text-align: center;">${data.kv_rating || 'N/A'}</td>
        <td style="padding: 12px; text-align: center;">${ktMeasured}</td>
        <td style="padding: 12px; text-align: center; font-weight: 600; color: #667eea;">${burstTorqueToWeight}</td>
        <td style="padding: 12px; text-align: center; ${burstTorqueError !== 'N/A' && parseFloat(burstTorqueError) < 10 ? 'color: #2e7d32;' : burstTorqueError !== 'N/A' && parseFloat(burstTorqueError) < 20 ? 'color: #f57c00;' : 'color: #c62828;'}">${burstTorqueError}${burstTorqueError !== 'N/A' ? '%' : ''}</td>
    `;

    return row;
}

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

        // Load and populate comparison table
        await loadComparisonTable(results);

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
                // In GitHub Actions deployment, everything is in the same directory
                window.location.href = `viewer.html?file=${encodeURIComponent(select.value)}`;
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
