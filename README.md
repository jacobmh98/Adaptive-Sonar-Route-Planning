# Adaptive-Sonar-Route-Planning

Required Python Packages:
numpy
matplotlib
shapely
scipy
rtree
tkinter
os
pickle
json
networkx
enum
ortools


# How to Use the UI

**Note**: All processing times mentioned below may vary depending on your system and the size of the file being processed.
**Note**: If obstacles are in the region, the greedy sorting does not decompose correctly.

## Steps:

### 1. Run the Application
Launch the Python script by running the "interface" Python class or the provided executable file.

### 2. Select a File
- Click the "Select File" button.  
- Navigate to the `test_data` folder (from the provided `.zip` file) and choose either a JSON or Pickle file.  
  - If a Pickle file is selected:  
    - Two plots will be displayed: one showing the region and another showing the decomposed sub-polygons stored in the file.  
  - If a JSON file is selected:  
    - Only the region plot will be displayed.

### 3. Decompose the Region
- If a Pickle file is selected:
  - Proceed to Step 4.  
- If a JSON file is selected:  
  - Choose a decomposition algorithm and click the "Decompose" button.  
      * Note: The "Greedy Recursive" algorithm can take more than 30 minutes for large regions (Antwerp, Great Belt bridge).

### 4. Optimize the Sub-Polygons
- If you selected "Greedy Recursive" or "Combination" as the decomposition algorithm, redundant sub-polygons may be created.  
- Click the "Optimize" button to merge these sub-polygons.  
    * Note: Optimization may take several minutes for large regions.

### 5. Set Sonar Path Parameters
- Enter a path width (in meters) for the sonar. The value must be greater than 0.  
- Enter a path overlap distance (in meters). This creates an overlap in the covered area. The value must be:  
  - Greater than or equal to 0.  
  - Less than the chosen path width.

### 6. Choose a Sorting Method
- Select a sorting method to determine the visiting order of the decomposed sub-polygons.

### 7. Include Coverage Plot (Optional)
- Check the box to include a coverage plot.  
  - This plot shows the covered area, overlap, and outlying regions for the generated path.

### 8. Inspect the Results
- View the generated plots and path data.  
- Use the plot toolbar to zoom, move, or save the plot.

### 9. Save Data (Optional)
- Click the "Save Data" button to save the generated plot as a Pickle file.
- Only the region and the decomposed sub-polygons of the plot that is currently being shown is saved.
- Enter a filename for the saved object. You can reload this file later.

### 10. Redo Options
#### Redo Decomposition
- Select a decomposition algorithm and repeat from Step 3.

#### Redo Path Generation
- Navigate to the plot showing the decomposed sub-polygons and repeat from Step 5.

#### Redo Region Selection
- Select a new file and repeat from Step 2.
