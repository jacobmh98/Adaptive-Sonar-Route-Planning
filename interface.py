import copy
import os
import pickle
import sys
import time
from time import perf_counter
import logging
import traceback

import plot_cpp
import plot_sorting
from cpp_alternative_path_finders import compute_spiral_path
from cpp_connect_path import connect_path, remove_duplicate_points_preserve_order
from cpp_path_planning import multi_intersection_planning
from decomposition import sum_of_widths, remove_collinear_vertices, optimize_polygons
from optimized_sweep_line import optimized_sweep_line
from plot_cpp import plot_multi_polys_path, plot_coverage_areas
from sorting_dfs_adjacency_graph import solve_dfs
from sorting_tsp_centroid import solve_centroid_tsp
from sorting_tsp_greedy import solve_greedy_tsp_sorting
from cpp_path_data import compute_path_data
from load_data import get_region, generate_new_data
from obstacles import plot_obstacles, decompose_sweep_line, merge_filtered_sub_polygons, find_bounding_polygons
from tkinter import *
from tkinter import filedialog
from datetime import datetime
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

# Define global variables
file_path = None
plots = []
current_plot_index = 0
region = None
obstacles = None
sub_polygons_list = []
stats = []
initial_xlim = None
initial_ylim = None
initial_data = None
canvas_list = []
toolbar_list = []

def reset(file_path_local = None):
    """ Reset the global variables """
    global file_path, plots, current_plot_index, stats, region, obstacles, sub_polygons_list, initial_xlim, initial_ylim, canvas_list, toolbar_list, canvas_frame

    file_path = file_path_local
    plots = []
    current_plot_index = 0
    stats = []
    region = None
    obstacles = None
    sub_polygons_list = []
    initial_xlim = None
    initial_ylim = None
    canvas_list = []
    toolbar_list = []

    for widget in canvas_frame.winfo_children():
        widget.destroy()

def select_file():
    global region, initial_xlim, initial_ylim, obstacles

    file_path = filedialog.askopenfilename(
        title="Select a File",
        filetypes=(("All Files", "*.*"),)
    )

    if file_path:
        reset(file_path)
        # Get the file extension
        file_name, file_extension = os.path.splitext(file_path)

        if file_extension == '.json':
            # Loading the region from a JSON file
            region, obstacles = get_region(file_path)
            fig = plot_obstacles([region], obstacles, False)
            plots.append(fig)

            canvas = FigureCanvasTkAgg(fig, master=canvas_frame)
            toolbar = NavigationToolbar2Tk(canvas, canvas_frame)
            canvas_list.append(canvas)
            toolbar_list.append(toolbar)

            sub_polygons_list.append(None)
            stats.append(None)

            initial_xlim = fig.get_axes()[0].get_xlim()
            initial_ylim = fig.get_axes()[0].get_ylim()

            update_plot()

        elif file_extension == '.pkl':

            # Loading the region and decomposition from a pickle file
            with open(f'{file_path}', "rb") as file:
                loaded_data = pickle.load(file)

            region = loaded_data[-3]
            obstacles = loaded_data[-2]
            statistics = loaded_data[-1]
            sub_polygons = loaded_data[:-3]

            fig = plot_obstacles([region], obstacles, False)
            plots.append(fig)

            canvas = FigureCanvasTkAgg(fig, master=canvas_frame)
            toolbar = NavigationToolbar2Tk(canvas, canvas_frame)
            canvas_list.append(canvas)
            toolbar_list.append(toolbar)

            stats.append(None)
            sub_polygons_list.append(None)

            initial_xlim = fig.get_axes()[0].get_xlim()
            initial_ylim = fig.get_axes()[0].get_ylim()

            fig = plot_obstacles(sub_polygons, obstacles, False)
            plots.append(fig)

            canvas = FigureCanvasTkAgg(fig, master=canvas_frame)
            toolbar = NavigationToolbar2Tk(canvas, canvas_frame)
            canvas_list.append(canvas)
            toolbar_list.append(toolbar)

            stats.append(statistics)
            sub_polygons_list.append(sub_polygons)

            update_plot()
        else:
            # Handle non compatible file-types
            None

def share_x_coordinate(region, obstacles):
    """ Test if any vertex in the region and obstacles shares an x-coordinate """
    seen_x = set()

    for v in region.vertices:
        if v.x in seen_x:
            return True
        else:
            seen_x.add(v.x)

    for o in obstacles:
        for v in o.vertices:
            if v.x in seen_x:
                return True
            else:
                seen_x.add(v.x)

    return False

def rotate_system(region, obstacles):
    """ Rotate the coordinate system until no vertices share an x-coordinate """
    angle_deg = 0

    while share_x_coordinate(region, obstacles):
        angle_deg += 1

        region.rotate(angle_deg)

        for o in obstacles:
            o.rotate(angle_deg)

    return region, obstacles, angle_deg

def rotate_system_back(sub_polygons, obstacles, angle_deg):
    for p in sub_polygons:
        p.rotate(-angle_deg)

    for o in obstacles:
        o.rotate(-angle_deg)

def decompose():
    """ When user presses the decompose button """
    global obstacles, current_plot_index

    if region and obstacles is not None:
        #plot_obstacles([region], obstacles, True, 'test')
        if len(obstacles) > 0 and decomposition_variable.get() == 'Combination':
            start_time = perf_counter()
            rotated_region, rotated_obstacles, angle_deg = rotate_system(copy.deepcopy(region),
                                                                         copy.deepcopy(obstacles))

            sub_polygons = generate_new_data(rotated_region)

            # Divide the sub-polygons into clusters that are affected by the obstacles
            sub_polygons_filtered_masks = []
            sub_polygons_filtered = []
            obstacles_affected = []

            for o in rotated_obstacles:
                filtered_mask, filtered = find_bounding_polygons(sub_polygons, o)
                common_found = False

                # plot_obstacles(filtered, obstacles, False)

                for index, mask in enumerate(sub_polygons_filtered_masks):
                    for i in mask:
                        for j in filtered_mask:
                            if i == j:
                                common_found = True
                                break

                        if common_found:
                            break

                    if common_found:
                        for i, p in enumerate(filtered_mask):
                            if p not in mask:
                                mask.append(p)
                                sub_polygons_filtered[index].append(filtered[i])

                            if o not in obstacles_affected[index]:
                                obstacles_affected[index].append(o)

                if not common_found:
                    sub_polygons_filtered_masks.append(filtered_mask)
                    sub_polygons_filtered.append(filtered)
                    obstacles_affected.append([o])

            # Merge each cluster into a single polygon and decompose it using sweep line algorithm
            decomposed_polygons = []
            extracted_sub_polygons = []
            extracted_sub_polygons_mask = []
            dont_include_mask = []

            for list in sub_polygons_filtered_masks:
                for p in list:
                    dont_include_mask.append(p)

            for i, filtered in enumerate(sub_polygons_filtered):
                sub_polygons_extract, merged_sub_polygon = merge_filtered_sub_polygons(copy.deepcopy(filtered),
                                                                                       copy.deepcopy(sub_polygons),
                                                                                       sub_polygons_filtered_masks[i])

                merged_sub_polygon_decomposed = decompose_sweep_line(merged_sub_polygon, obstacles_affected[i])
                decomposed_polygons += merged_sub_polygon_decomposed

                for p in sub_polygons_extract:
                    if p not in extracted_sub_polygons_mask and p not in dont_include_mask:
                        extracted_sub_polygons_mask.append(p)
                        extracted_sub_polygons.append(sub_polygons[p])

                plot_obstacles(extracted_sub_polygons + [merged_sub_polygon], rotated_obstacles, False)

            # Combining all the decomposed sub-polygons with obstacles
            combined_polygons = extracted_sub_polygons + decomposed_polygons

            rotate_system_back(combined_polygons, rotated_obstacles, angle_deg)

            end_time = perf_counter()

            fig = plot_obstacles(combined_polygons, rotated_obstacles, False)

            sub_polygons_list.append(combined_polygons)

            plots.append(fig)

            decomposition_stats = {
                'type': 'decomposition_statistics',
                'method': 'Combination',
                'number_of_polygons': len(combined_polygons),
                'sum_of_widths': sum_of_widths(combined_polygons),
                'execution_time': end_time - start_time
            }

            stats.append(decomposition_stats)
        elif decomposition_variable.get() == 'Greedy Recursive' or (len(obstacles) == 0 and decomposition_variable.get() == 'Combination'):
            start_time = perf_counter()
            # Decompose the region without considering obstacles
            sub_polygons = generate_new_data(copy.deepcopy(region))

            end_time = perf_counter()

            sub_polygons_list.append(sub_polygons)
            fig = plot_obstacles(sub_polygons, obstacles, False)
            plots.append(fig)

            decomposition_stats = {
                'type': 'decomposition_statistics',
                'method': 'Greedy Recursive',
                'number_of_polygons': len(sub_polygons),
                'sum_of_widths': sum_of_widths(sub_polygons),
                'execution_time': end_time - start_time
            }
            stats.append(decomposition_stats)
        elif decomposition_variable.get() == 'Sweep Line':
            start_time = perf_counter()
            # Decompose the region without considering obstacles
            rotated_region, rotated_obstacles, angle_deg = rotate_system(copy.deepcopy(region), copy.deepcopy(obstacles))

            sub_polygons = decompose_sweep_line(rotated_region, rotated_obstacles)
            rotate_system_back(sub_polygons, rotated_obstacles, angle_deg)
            end_time = perf_counter()

            sub_polygons_list.append(sub_polygons)
            fig = plot_obstacles(sub_polygons, rotated_obstacles, False)
            plots.append(fig)

            decomposition_stats = {
                'type': 'decomposition_statistics',
                'method': 'Sweep Line',
                'number_of_polygons': len(sub_polygons),
                'sum_of_widths': sum_of_widths(sub_polygons),
                'execution_time': end_time - start_time
            }

            stats.append(decomposition_stats)
        elif decomposition_variable.get() == 'Optimized Sweep Line':
            start_time = perf_counter()
            sub_polygons = optimized_sweep_line(copy.deepcopy(region), copy.deepcopy(obstacles))
            end_time = perf_counter()

            sub_polygons_list.append(sub_polygons)
            fig = plot_obstacles(sub_polygons, obstacles, False)
            plots.append(fig)

            decomposition_stats = {
                'type': 'decomposition_statistics',
                'method': 'Optimized Sweep Line',
                'number_of_polygons': len(sub_polygons),
                'sum_of_widths': sum_of_widths(sub_polygons),
                'execution_time': end_time - start_time
            }

            stats.append(decomposition_stats)
        
        current_plot_index = len(plots) - 1
        update_plot()

def update_plot():
    global current_plot_index, canvas, toolbar, canvas_frame

    #fig.get_axes()[0].set_xlim(initial_xlim)
    #fig.get_axes()[0].set_ylim(initial_ylim)
    #fig.tight_layout()
    #fig.get_axes()[0].set_aspect('equal')

    if current_plot_index >= len(canvas_list):
        fig = plots[current_plot_index]

        canvas = FigureCanvasTkAgg(fig, master=canvas_frame)
        toolbar = NavigationToolbar2Tk(canvas, canvas_frame)
        canvas_list.append(canvas)
        toolbar_list.append(toolbar)

    for i, (canvas, toolbar) in enumerate(zip(canvas_list, toolbar_list)):
        if i == current_plot_index:
            canvas.get_tk_widget().pack(side=TOP, fill=BOTH, expand=True)
            toolbar.pack(side=BOTTOM, fill=X)
        else:
            canvas.get_tk_widget().pack_forget()
            toolbar.pack_forget()

    #canvas = canvas_list[current_plot_index]
    #canvas.draw()
    #canvas.get_tk_widget().pack(side='top', fill='both', expand=True)

    # Add a matplotlib navigation toolbar
    ##toolbar = toolbar_list[current_plot_index]
    #toolbar.update()
    #toolbar.pack(side='bottom', fill='x')


    """        # Update the canvas with the new figure
        canvas.figure = fig
        canvas.draw()  # Redraw the canvas efficiently

    # Update the toolbar to reflect the new figure
    toolbar.destroy()  # Destroy the old toolbar
    toolbar = NavigationToolbar2Tk(canvas, canvas_frame)
    toolbar.update()
    toolbar.pack(side='bottom', fill='x')"""

    # Remove the old canvas widget
    """canvas.get_tk_widget().pack_forget()

    canvas = FigureCanvasTkAgg(fig, master=canvas_frame)
    canvas.draw()
    canvas.get_tk_widget().pack()

    # Reinitialize the toolbar with the new canvas
    toolbar.destroy()  # Remove the old toolbar
    toolbar = NavigationToolbar2Tk(canvas, canvas_frame)
    toolbar.pack(side=TOP, fill=X)"""

    update_stats()

def next_plot():
    """ Function to show the next plot """
    global current_plot_index
    if len(plots) > 0:
        current_plot_index = (current_plot_index + 1) % len(plots)
        update_plot()

def prev_plot():
    """ Function to show the previous plot """
    global current_plot_index
    if len(plots) > 0:
        current_plot_index = (current_plot_index - 1) % len(plots)
        update_plot()

def update_stats():
    global plot_index, scrollable_content

    # Clear any previous content from the scrollable_content
    for widget in scrollable_content.winfo_children():
        widget.destroy()

    plot_index.config(text=f"{current_plot_index + 1} / {len(plots)}")

    stats_dict = stats[current_plot_index]

    if stats_dict is None:
        # Title for the chosen file
        Label(scrollable_content, text="Chosen File", font=("Arial", 12, "bold"), anchor="w").grid(row=0, column=0, padx=0, pady=5, sticky="w")

        # Display the file name if a file is chosen
        if file_path:
            file_name = os.path.basename(file_path)  # Extract the file name from the path
            Label(scrollable_content, text=file_name, font=("Arial", 10), anchor="w").grid(row=1, column=0, padx=(0,5), pady=2, sticky="w")
        else:
            Label(scrollable_content, text="No File Selected", font=("Arial", 10), anchor="w").grid(row=1, column=0, padx=(0,5), pady=2, sticky="w")

    elif stats_dict['type'] == 'coverage_statistics':
        # Titles for columns
        general_title = Label(scrollable_content, text="General Data", font=("Arial", 12, "bold"), anchor="w")
        path_title = Label(scrollable_content, text="Path Data", font=("Arial", 12, "bold"), anchor="w")
        coverage_title = Label(scrollable_content, text="Coverage Data", font=("Arial", 12, "bold"), anchor="w")

        general_title.grid(row=0, column=0, sticky="w", padx=(0,5), pady=5)
        path_title.grid(row=0, column=1, sticky="w", padx=(0,5), pady=5)
        coverage_title.grid(row=0, column=2, sticky="w", padx=(0,5), pady=5)

        # General data
        general_data = [
            f"Sorting Method: {stats_dict['sorting_variable']}",
            f"Path Width: {stats_dict['path_width']}m",
            f"Overlap Distance: {stats_dict['overlap_distance']}m",
            f"Total Execution Time: {round(stats_dict['total_execution_time'], 2)}s",
            f"Sorting Time: {round(stats_dict['sorting_time'], 2)}s",
        ]
        for i, text in enumerate(general_data, start=1):
            Label(scrollable_content, text=text, anchor="w").grid(row=i, column=0, sticky="w", padx=(0,5), pady=1)

        # Path data
        path_data = [
            f"Total Distance: {round(stats_dict['total_distance'], 2)}m",
            f"Path Distance: {round(stats_dict['path_distance'], 2)}m",
            f"Transit Distance: {round(stats_dict['transit_distance'], 2)}m",
            f"Total number of Turns: {stats_dict['total_turns']}",
            f"Sharp Turns (<45°): {stats_dict['hard_turns']}",
            f"Moderate Turns (45° - 90°): {stats_dict['medium_turns']}",
            f"Gentle Turns (>90°): {stats_dict['soft_turns']}"
        ]
        for i, text in enumerate(path_data, start=1):
            Label(scrollable_content, text=text, anchor="w").grid(row=i, column=1, sticky="w", padx=(0,5), pady=1)

        # Coverage data
        coverage_data = [
            f"Coverage Percentage: {round(stats_dict['coverage_percentage'], 2)}%",
            f"Covered Area: {round(stats_dict['covered_area'].area, 2)}m²",
            f"Outlying Area: {round(stats_dict['outlying_area'].area, 2)}m²",
            f"Overlapped Area: {round(stats_dict['overlapped_area'].area, 2)}m²",
        ]
        for i, text in enumerate(coverage_data, start=1):
            Label(scrollable_content, text=text, anchor="w").grid(row=i, column=2, sticky="w", padx=(0,5), pady=1)

    elif stats_dict['type'] == 'decomposition_statistics':
        # Title for decomposition statistics
        title = Label(scrollable_content, text="Decomposition Statistics", font=("Arial", 12, "bold"), anchor="w")
        title.grid(row=0, column=0, sticky="w", padx=(0,5), pady=5)

        # Decomposition data
        decomposition_data = [
            f"Method: {stats_dict['method']}",
            f"Number of Polygons: {stats_dict['number_of_polygons']}",
            f"Sum of Widths: {round(stats_dict['sum_of_widths'], 2)}",
            f"Execution Time: {round(stats_dict['execution_time'], 2)}"
        ]
        for i, text in enumerate(decomposition_data, start=1):
            Label(scrollable_content, text=text, anchor="w").grid(row=i, column=0, sticky="w", padx=(0,5), pady=1)

def path_planner():
    global current_plot_index, sorting_variable, show_coverage_var, \
           use_transit_lines_var, hide_plot_legend_var, hide_sub_polygon_indices_var

    if len(sub_polygons_list) != 0:
        sub_polygons = sub_polygons_list[current_plot_index]

        if sub_polygons is not None:
            path_width_value = path_width_entry.get()
            overlap_value = overlap_distance_entry.get()
            print(f'{path_width_value=}')

            if path_width_value:  # Check if it's not empty
                try:
                    # Convert to integer
                    global chosen_path_width, chosen_overlap_distance
                    chosen_path_width = float(path_width_value)
                    if chosen_path_width == 0:
                        chosen_path_width = 10
                        print("0 not valid path width, it is set to 10")

                    chosen_overlap_distance = float(overlap_value)

                    # Removing collinear vertices
                    removed_col_sub_polygons = []
                    for poly in sub_polygons:
                        removed_col_sub_polygons.append(remove_collinear_vertices(poly))

                    total_start_time = time.time()
                    intersections = multi_intersection_planning(removed_col_sub_polygons, chosen_path_width, chosen_overlap_distance)
                    sorting_var = sorting_variable.get()

                    if len(removed_col_sub_polygons) < 3:
                        sorting_var = "Unordered"

                    if sorting_var == 'DFS':
                        print("DFS")
                        sorting_start_time = time.time()
                        sorted_sub_polygons, sorted_col_removed_sub_polygons, sorted_intersections = solve_dfs(removed_col_sub_polygons, intersections)
                        sorting_end_time = time.time()
                        total_sorting_time = sorting_end_time - sorting_start_time

                    elif sorting_var == 'TSP Centroid':
                        print("Centroids")
                        sorting_start_time = time.time()
                        sorted_sub_polygons, sorted_col_removed_sub_polygons, sorted_intersections = solve_centroid_tsp(removed_col_sub_polygons, intersections)
                        sorting_end_time = time.time()
                        total_sorting_time = sorting_end_time - sorting_start_time

                    elif sorting_var == 'TSP Intra Regional':
                        print("Intra Regional")
                        sorting_start_time = time.time()
                        sorted_sub_polygons, sorted_col_removed_sub_polygons, sorted_intersections = solve_greedy_tsp_sorting(removed_col_sub_polygons, intersections)
                        sorting_end_time = time.time()
                        total_sorting_time = sorting_end_time - sorting_start_time
                    else:
                        print("Unordered")
                        sorted_sub_polygons = sub_polygons
                        sorted_col_removed_sub_polygons = removed_col_sub_polygons
                        sorted_intersections = intersections
                        total_sorting_time = 0

                    # Storing data as pkl objects.
                    #data_dir = "C:/Users/andre/Documents/Adaptive-Sonar-Route-Planning/comparison_test_files"
                    #os.makedirs(data_dir, exist_ok=True)

                    #file_path_region = os.path.join(data_dir, "test.pkl")
                    #file_path_polys = os.path.join(data_dir, "test.pkl")

                    # Save the 'region' object
                    #with open(file_path_region, 'wb') as f:
                    #    pickle.dump(region, f)

                    # Save the 'sorted_sub_polygons' object
                    #with open(file_path_polys, 'wb') as f:
                    #    pickle.dump(sorted_sub_polygons, f)


                    # Computing path
                    #plot_cpp.plot_single_polygon_with_intersections(sorted_sub_polygons, sorted_intersections)
                    path, transit_flags = connect_path(sorted_sub_polygons, sorted_intersections,
                                                                        region, obstacles)
                    # Removing duplicate points from the path OBS: Creates some errors with hard edges rerouting (same vertices used to navigate around obstacles/hard edges)
                    #path, transit_flags = remove_duplicate_points_preserve_order(path, transit_flags)

                    # Ending timer and computing total execution time
                    total_end_time = time.time()
                    total_execution_time = total_end_time - total_start_time

                    # Choose to not differentiate from transit and normal path points
                    if not use_transit_lines_var.get():
                        transit_flags = [None] * len(transit_flags)

                    # No transit lines for a single polygon - Might be unnecessary since none should be added
                    if len(sorted_col_removed_sub_polygons) == 1:
                        transit_flags = [None] * len(transit_flags)

                    # Set to true to check spiral path, only works for rectangles.
                    spiral_path = False

                    if spiral_path:
                        boundary_box = sorted_sub_polygons[0].compute_boundary()
                        path = compute_spiral_path(sorted_sub_polygons[0], chosen_path_width, boundary_box)
                        path = remove_duplicate_points_preserve_order(path)
                        transit_flags = [None] * len(path)

                    # Computing plot for path
                    fig_path = plot_multi_polys_path(chosen_path_width, sorted_sub_polygons, path, obstacles,
                                                              False, transit_flags,
                                                              hide_plot_legend_var.get(), hide_sub_polygon_indices_var.get())
                    # Computing data about path
                    stats_dict = compute_path_data(region, path, transit_flags, chosen_path_width,
                                                                 obstacles, total_execution_time)

                    stats_dict['total_execution_time'] = total_execution_time
                    stats_dict['sorting_variable'] = sorting_variable.get()
                    stats_dict['sorting_time'] = total_sorting_time
                    stats_dict['path_width'] = chosen_path_width
                    stats_dict['overlap_distance'] = chosen_overlap_distance
                    stats.append(stats_dict)
                    sub_polygons_list.append(None)
                    plots.append(fig_path)

                    #TODO set canvas here
                    canvas = FigureCanvasTkAgg(fig_path, master=canvas_frame)
                    toolbar = NavigationToolbar2Tk(canvas, canvas_frame)
                    canvas_list.append(canvas)
                    toolbar_list.append(toolbar)

                    # Checking if user wants to see coverage plot
                    if show_coverage_var.get():
                        fig_coverage = plot_coverage_areas(sorted_sub_polygons,
                                                                    stats_dict["covered_area"],
                                                                    stats_dict["overlapped_lines"],
                                                                    stats_dict["outlier_lines"],
                                                                    path, transit_flags, hide_plot_legend_var.get(),
                                                                    hide_sub_polygon_indices_var.get())

                        stats_dict['total_execution_time'] = total_execution_time
                        stats_dict['sorting_variable'] = sorting_variable.get()
                        stats_dict['sorting_time'] = total_sorting_time
                        stats_dict['path_width'] = chosen_path_width
                        stats_dict['overlap_distance'] = chosen_overlap_distance
                        stats.append(stats_dict)
                        sub_polygons_list.append(None)
                        plots.append(fig_coverage)

                        # TODO set canvas here
                        canvas = FigureCanvasTkAgg(fig_coverage, master=canvas_frame)
                        toolbar = NavigationToolbar2Tk(canvas, canvas_frame)
                        canvas_list.append(canvas)
                        toolbar_list.append(toolbar)

                    current_plot_index = len(plots) - 1
                    update_plot()


                except Exception as e:  # Catch all exceptions
                    print(str(e))  # Print the exception message
                    traceback.print_exc()  # Print the full traceback for debugging

def is_float(value):
    try:
        float(value)
        return True
    except ValueError:
        return False

def validate_integer_input(new_value):
    # Allow empty input (so user can delete characters) or integer
    if new_value == "" or is_float(new_value):
        return True
    else:
        return False

def validate_path_width_input(new_value):
    # Allow empty input (so user can delete characters) or integer
    if new_value == "" or is_float(new_value):
        return True
    else:
        return False

def save_data():
    global file_path

    if file_path is not None:
        if len(sub_polygons_list) > 0 and sub_polygons_list[current_plot_index] is not None:
            # Get the file extension
            file_name = os.path.splitext(os.path.basename(file_path))[0]

            # Open a file dialog to choose the location and filename to save
            save_file_path = filedialog.asksaveasfilename(defaultextension=".pkl",
                                                      filetypes=[("Pickle Files", "*.pkl")],
                                                      initialfile=file_name)


            if save_file_path is not None and save_file_path != '':
                data = sub_polygons_list[current_plot_index] + [region] + [obstacles] + [stats[current_plot_index]]

                #print(f'{file_name=}')

                with open(f'{save_file_path}', 'wb') as file:
                    pickle.dump(data, file)
                print('Data saved successfully')

def setup_plot_pane():
    """ Set up the initial canvas with the first plot """
    global canvas, plot_index, stats_frame, scrollable_content, toolbar, stats_canvas, canvas_frame, scrollbar
   # global canvas, plot_index, stats_label, toolbar, canvas_frame

    canvas_frame = Frame(plot_pane, bg='white', width='600', height='480')
    canvas_frame.pack(fill='both', expand=True)

    # Frame to contain the buttons
    button_frame = Frame(plot_pane)
    button_frame.pack(fill="x", padx=10, pady=2)

    prev_btn = Button(button_frame, text='Previous', command=prev_plot)
    prev_btn.pack(side="left", anchor='w')

    next_btn = Button(button_frame, text='Next', command=next_plot)
    next_btn.pack(side='right', anchor='e')

    plot_index = Label(button_frame, text=f'{current_plot_index} / {len(plots)}', font=("Arial", 10))
    plot_index.place(relx=0.5, rely=0.5, anchor="center")  # Anchor to center

    Button(button_frame, text='Save Data', command=save_data).place(relx=0.6, rely=0.5, anchor="center")

    # Create a frame for stats with scrollbar
    stats_frame = Frame(plot_pane)
    stats_frame.pack(fill="both", expand=True, padx=10, pady=1)

    # Add canvas and scrollbar for scrolling
    stats_canvas = Canvas(stats_frame)
    stats_canvas.pack(side="left", fill="both", expand=True)

    scrollbar = Scrollbar(stats_frame, orient="vertical", command=stats_canvas.yview)
    scrollbar.pack(side="right", fill="y")

    # Configure the canvas to work with the scrollbar
    stats_canvas.configure(yscrollcommand=scrollbar.set)

    # Frame inside the canvas for the actual stats content
    scrollable_content = Frame(stats_canvas)
    stats_canvas.create_window((0, 0), window=scrollable_content, anchor="nw")

    # Update scroll region whenever the content changes
    def update_scrollregion(event=None):
        stats_canvas.update_idletasks()
        bbox = stats_canvas.bbox("all")
        stats_canvas.configure(scrollregion=bbox)

        # Check if the content fits inside the canvas
        content_height = bbox[3] - bbox[1] if bbox else 0  # Height of content
        canvas_height = stats_canvas.winfo_height()        # Height of visible canvas

        if content_height <= canvas_height:  # If content fits, disable scrolling
            stats_canvas.unbind_all("<MouseWheel>")
            stats_canvas.configure(yscrollcommand=None)
            scrollbar.pack_forget()  # Hide the scrollbar
        else:  # If content exceeds, enable scrolling
            stats_canvas.bind_all("<MouseWheel>", lambda event: stats_canvas.yview_scroll(-1 * int(event.delta / 120), "units"))
            stats_canvas.configure(yscrollcommand=scrollbar.set)
            scrollbar.pack(side="right", fill="y")  # Show the scrollbar

    # Bind the <Configure> event of the `scrollable_content` frame
    scrollable_content.bind("<Configure>", update_scrollregion)

def optimize():
    global current_plot_index

    if len(plots) > 0:
        sub_polygons = sub_polygons_list[current_plot_index]

        if sub_polygons is not None:
            optimized_polygons = optimize_polygons(copy.deepcopy(sub_polygons))

            if len(sub_polygons) != len(optimized_polygons):
                fig = plot_obstacles(optimized_polygons, obstacles, False)

                decomposition_stats = {
                    'type': 'decomposition_statistics',
                    'method': f'{stats[current_plot_index]['method']} (Optimized)',
                    'number_of_polygons': len(optimized_polygons),
                    'sum_of_widths': sum_of_widths(optimized_polygons)
                }

                sub_polygons_list.append(optimized_polygons)
                plots.append(fig)
                stats.append(decomposition_stats)

                current_plot_index = len(plots) - 1
                update_plot()

def setup_option_pane():
    """ Creating the options pane """
    global label, decomposition_variable, path_width_entry, overlap_distance_entry, sorting_variable, \
           show_coverage_var, use_transit_lines_var, hide_plot_legend_var, hide_sub_polygon_indices_var

    Label(options_pane, text='Select File', font=("Arial", 14)).pack(anchor='w')
    Button(options_pane, text="Select File", command=select_file).pack(anchor='w')

    Label(options_pane, text='Decomposition Algorithm', font=('Arial, 14')).pack(anchor='w', pady=(15, 0))

    decomposition_variable = StringVar(value='Optimized Sweep Line')
    rb8 = Radiobutton(options_pane, text='Optimized Sweep Line', variable=decomposition_variable, value='Optimized Sweep Line',)
    rb1 = Radiobutton(options_pane, text='Greedy Recursive', variable=decomposition_variable, value='Greedy Recursive')
    rb2 = Radiobutton(options_pane, text='Sweep Line', variable=decomposition_variable, value='Sweep Line')
    rb3 = Radiobutton(options_pane, text='Combination', variable=decomposition_variable, value='Combination')
    rb8.pack(anchor='w')
    rb1.pack(anchor='w')
    rb2.pack(anchor='w')
    rb3.pack(anchor='w')

    Button(options_pane, text="Decompose", command=decompose).pack(anchor='w')
    Button(options_pane, text='Optimize', command=optimize).pack(anchor='w', pady=(5, 0))

    Label(options_pane, text='Path Width', font=('Arial, 14')).pack(anchor='w', pady=(15, 0))

    path_width_entry = Entry(options_pane, validate="key", validatecommand=(validate_path_width, "%P"))
    path_width_entry.pack(anchor='w')
    path_width_entry.insert(0, '10')

    Label(options_pane, text='Path Overlap Distance', font=('Arial, 14')).pack(anchor='w', pady=(10, 0))
    overlap_distance_entry = Entry(options_pane, validate="key", validatecommand=(validate_cmd, "%P"))
    overlap_distance_entry.pack(anchor='w')
    overlap_distance_entry.insert(0, '0')

    Label(options_pane, text='Sorting Method', font=('Arial, 14')).pack(anchor='w', pady=(15, 0))

    sorting_variable = StringVar(value='Unordered')
    rb4 = Radiobutton(options_pane, text='Unordered', variable=sorting_variable, value='Unordered')
    rb5 = Radiobutton(options_pane, text='DFS', variable=sorting_variable, value='DFS')
    rb6 = Radiobutton(options_pane, text='TSP Centroid', variable=sorting_variable, value='TSP Centroid')
    rb7 = Radiobutton(options_pane, text='TSP Intra Regional', variable=sorting_variable, value='TSP Intra Regional')
    rb4.pack(anchor='w')
    rb5.pack(anchor='w')
    rb6.pack(anchor='w')
    rb7.pack(anchor='w')

    Label(options_pane, text='Path Planner', font=('Arial, 14')).pack(anchor='w', pady=(15, 0))

    show_coverage_var = IntVar()
    use_transit_lines_var = IntVar()#value=1)
    hide_plot_legend_var = IntVar()#value=1)
    hide_sub_polygon_indices_var = IntVar()

    Checkbutton(options_pane, text="Show Coverage Plot", variable=show_coverage_var).pack(anchor='w')
    Checkbutton(options_pane, text="Use Transit Lines", variable=use_transit_lines_var).pack(anchor='w')
    Checkbutton(options_pane, text="Hide Plot legend", variable=hide_plot_legend_var).pack(anchor='w')
    Checkbutton(options_pane, text="Hide Sub Polygon Indices", variable=hide_sub_polygon_indices_var).pack(anchor='w')

    Button(options_pane, text='Create Path', command=path_planner).pack(anchor='w')

# Initialize main Tkinter window
root = Tk()
root.title('Adaptive Route Planning')

# Register the validation function with Tkinter
validate_cmd = root.register(validate_integer_input)
validate_path_width = root.register(validate_path_width_input)

# Set the window size to the screen size (maximized window)
screen_width = root.winfo_screenwidth()
screen_height = root.winfo_screenheight()
root.geometry(f"{screen_width}x{screen_height}+0+0")

paned_window = PanedWindow(root, orient=HORIZONTAL)
paned_window.pack(fill=BOTH, expand=1)

plot_pane = Frame(paned_window)
paned_window.add(plot_pane)

options_pane = Frame(paned_window)
paned_window.add(options_pane)

# Set up the plot pane
setup_plot_pane()

# Set up the option pane
setup_option_pane()

# Initialize the plot list and the index
#plots = create_plots()
#current_plot_index = 0

def on_closing():
    """Handle the application closing."""
    root.quit()  # Stops the Tkinter mainloop
    root.destroy()  # Destroys all Tkinter widgets and exits the app
    sys.exit()  # Ensures the Python process exits

root.protocol("WM_DELETE_WINDOW", on_closing)

log_data = False
if log_data:
    # Create a logs directory if it doesn't exist
    os.makedirs("logs", exist_ok=True)

    # Generate a unique log filename with a timestamp
    log_filename = f"logs/application_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"

    # Configure logging
    logging.basicConfig(
        filename=log_filename,
        filemode="a",
        level=logging.DEBUG,
        format="%(asctime)s - %(levelname)s - %(message)s",
    )

"""# Redirect stdout and stderr to logging
class StreamToLogger:
    def __init__(self, logger, level):
        self.logger = logger
        self.level = level


        def write(self, message):
            if message.strip():  # Avoid blank lines
                self.logger.log(self.level, message.strip())

        def flush(self):
            pass  # Required for compatibility with file-like objects


    sys.stdout = StreamToLogger(logging.getLogger("stdout"), logging.INFO)
    sys.stderr = StreamToLogger(logging.getLogger("stderr"), logging.ERROR)

sys.stdout = StreamToLogger(logging.getLogger("stdout"), logging.INFO)
sys.stderr = StreamToLogger(logging.getLogger("stderr"), logging.ERROR)"""

if __name__ == "__main__":
    root.mainloop()