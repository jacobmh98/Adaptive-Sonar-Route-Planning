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


Using the UI:
1. Run the "interface" python class or the executable project file.
2. Click "Select File" and navigate to the "test_data" folder from the .zip file, and pick either a JSON or Pickle file. <br>
   2.1 If a Pickle file chosen, 2 plots will be showed, one of the region, the other is the stored decomposed sub polygons.
   2.2 If a JSON file is chosen only the region is shown
3. If Pickle chosen, go to 4, else pick a decomposition algorithm and click the "Decompose" button. <br>
   3.1 Note that the "Greedy Recursive can take more than 30 minutes if a large region is chosen.
4. If "Greedy Recursive" or "Combination" can create redundant sub-polygons that can be merged using the "Optimize" button. <br>
   4.1 Note that the optimization of the sub polygons can take more than 5 minutes for large regions. TODO JACOB tid korrekt?
5. Write a path width for the sonar in meters, must be greater than 0
6. Write a path overlap distance in meters, this will create an overlap in the covered area, must be greater than 0 and less than the chosen path width.
7. Choose sorting method, this will determine the visiting order of the decomposed sub polygons.
   7.1 If "TSP Intra Regional" is chosen, then pick the number of iterations for it to run.
   7.2 0 iterations results in an undordered list.
   7.3 Each iteration takes roughly 30 seconds to compute, but can vary depending on number of sub polygons.
8. Click checkbox to include coverage plot, this is a seperate plot showing the path's covered, overlap and outlying area
9. Inspect the generated plots and path data, use plot toolbar to zoom, move or save the generated plot.
10. If wanted, click the "Save Data" button and write a filename to save the generated plot as a Pickle object that can be loaded again
11. If new decompostion method wanted, navigate back to the plot showing the region
12. Redo from step 3
13. If new path generation wanted, navigated to the plot showing the decomposed sub polygons
14. Redo from step 5
15. If new region wanted, select a new file and redo from 2
