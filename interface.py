import copy
import os
import pickle
import time
from tkinter import *
from tkinter import filedialog

import matplotlib
import networkx as nx
from matplotlib import pyplot as plt
from matplotlib.backends._backend_tk import NavigationToolbar2Tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

import cpp_connect_path
import cpp_path_planning
import decomposition
import plot_cpp
import sorting_dfs_adjacency_graph
import sorting_tsp_centroid
import sorting_tsp_intra_regional
from decomposition import sum_of_widths, optimize_polygons
from global_variables import number_of_tsp_trials
from load_data import get_region, generate_new_data
from obstacles import plot_obstacles, decompose_sweep_line, merge_filtered_sub_polygons, find_bounding_polygons
from cpp_path_data import compute_path_data, compute_covered_area_with_obstacles, compute_outlier_area, \
    compute_overlap_area
from sorting_dfs_adjacency_graph import create_adjacency

file_path = None
plots = []
current_plot_index = 0
region = None
obstacles = None
sub_polygons_list = []
stats = []

def reset():
    global file_path, plots, current_plot_index, stats, region, obstacles, sub_polygons_list

    file_path = None
    plots = []
    current_plot_index = 0
    stats = []
    region = None
    obstacles = None
    sub_polygons_list = []

def select_file():
    global file_path, region, obstacles

    reset()

    file_path = filedialog.askopenfilename(
        title="Select a File",
        filetypes=(("All Files", "*.*"),)
    )

    if file_path:
        label.config(text=f"{file_path}")

        # Get the file extension
        file_name, file_extension = os.path.splitext(file_path)

        if file_extension == '.json':
            region, obstacles = get_region(file_path)
            fig = plot_obstacles([region], obstacles, False)
            plots.append(fig)
            sub_polygons_list.append(None)
            stats.append(None)
            update_plot()
        elif file_extension == '.pkl':
            # Loading the list back from the file
            with open(f'{file_path}', "rb") as file:
                loaded_data = pickle.load(file)

            region = loaded_data[-3]
            obstacles = loaded_data[-2]
            statistics = loaded_data[-1]
            sub_polygons = loaded_data[:-3]

            fig = plot_obstacles([region], obstacles, False)
            plots.append(fig)
            stats.append(None)
            sub_polygons_list.append(None)

            fig = plot_obstacles(sub_polygons, obstacles, False)
            plots.append(fig)
            stats.append(statistics)
            sub_polygons_list.append(sub_polygons)

            update_plot()

def decompose():
    global current_plot_index
    if region is not None:
        if len(obstacles) > 0 and decomposition_variable.get() == 'Combination':
            sub_polygons = generate_new_data(copy.deepcopy(region))

            # Divide the sub-polygons into clusters that are affected by the obstacles
            sub_polygons_filtered_masks = []
            sub_polygons_filtered = []
            obstacles_affected = []

            for o in obstacles:
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

                # plot_obstacles(merged_sub_polygon_decomposed, obstacles, False)

                for p in sub_polygons_extract:
                    if p not in extracted_sub_polygons_mask and p not in dont_include_mask:
                        extracted_sub_polygons_mask.append(p)
                        extracted_sub_polygons.append(sub_polygons[p])

                plot_obstacles(extracted_sub_polygons + [merged_sub_polygon], obstacles, False)

            # Combining all the decomposed sub-polygons with obstacles
            combined_polygons = extracted_sub_polygons + decomposed_polygons
            fig = plot_obstacles(combined_polygons, obstacles, False)

            sub_polygons_list.append(combined_polygons)

            plots.append(fig)

            decomposition_stats = {
                'type': 'decomposition_statistics',
                'method': 'Sweep Line',
                'number_of_polygons': len(combined_polygons),
                'sum_of_widths': sum_of_widths(combined_polygons)
            }

            stats.append(decomposition_stats)
        elif decomposition_variable.get() == 'Greedy Recursive' or (len(obstacles) == 0 and decomposition_variable.get() == 'Combination'):
            # Decompose the region without considering obstacles
            sub_polygons = generate_new_data(copy.deepcopy(region))
            sub_polygons_list.append(sub_polygons)
            fig = plot_obstacles(sub_polygons, obstacles, False)
            plots.append(fig)



            decomposition_stats = {
                'type': 'decomposition_statistics',
                'method': 'Greedy Recursive',
                'number_of_polygons': len(sub_polygons),
                'sum_of_widths': sum_of_widths(sub_polygons)
            }

            stats.append(decomposition_stats)
        elif decomposition_variable.get() == 'Sweep Line':
            # Decompose the region without considering obstacles
            sub_polygons = decompose_sweep_line(copy.deepcopy(region), copy.deepcopy(obstacles))
            sub_polygons_list.append(sub_polygons)
            fig = plot_obstacles(sub_polygons, obstacles, False)
            plots.append(fig)

            decomposition_stats = {
                'type': 'decomposition_statistics',
                'method': 'Sweep Line',
                'number_of_polygons': len(sub_polygons),
                'sum_of_widths': sum_of_widths(sub_polygons)
            }

            stats.append(decomposition_stats)


        current_plot_index = len(plots) - 1

        update_plot()

def update_plot():
    global current_plot_index, canvas, toolbar, canvas_frame

    # Remove the old canvas widget
    canvas.get_tk_widget().pack_forget()

    canvas = FigureCanvasTkAgg(plots[current_plot_index], master=canvas_frame)
    canvas.draw()
    canvas.get_tk_widget().pack()

    # Reinitialize the toolbar with the new canvas
    toolbar.destroy()  # Remove the old toolbar
    toolbar = NavigationToolbar2Tk(canvas, canvas_frame)
    toolbar.pack(side=TOP, fill=X)

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
    global plot_index, stats_label
    plot_index.config(text=f"{current_plot_index + 1} / {len(plots)}")

    stats_dict = stats[current_plot_index]

    if stats_dict is None:
        stats_label.config(text=f'')
    elif stats_dict['type'] == 'coverage_statistics':
        str = (f'Sorting Method = {stats_dict['sorting_variable']} \n'
               f'Total Execution Time = {round(stats_dict['total_execution_time'], 2)} s \n'
               f'Sorting Time = {round(stats_dict['sorting_time'], 2)} s \n'
               f'Path Width = {stats_dict['path_width']} m \n'
               f'Overlap Distance = {stats_dict['overlap_distance']} m \n'
               f'Path Distance = {round(stats_dict["distance"], 2)} m \n'
               f'Total number of Turns = {stats_dict["total_turns"]} \n'
               f'Coverage Percentage = {round(stats_dict["coverage_percentage"], 2)} % \n'
               f'Covered Area = {round(stats_dict["covered_area"].area, 2)} m\u00B2 \n'
               f'Overlapped Area = {round(stats_dict["overlapped_area"].area, 2)} m\u00B2 \n'
               f'Outlying Area = {round(stats_dict["outlying_area"].area, 2)} m\u00B2 \n')
               #f'Hard Turns (<45) = {stats_dict["hard_turns"]} \n'
               #f'Medium Turns (45-90) = {stats_dict["medium_turns"]} \n'
               #f'Soft Turns (>90) = {stats_dict["soft_turns"]}')

        stats_label.config(text=str,
                            justify="left",  # Justify text to the left
                            anchor="w")
    elif stats_dict['type'] == 'decomposition_statistics':
        str = (f'Method = {stats_dict["method"]} \n'
               f'Number of Polygons = {stats_dict["number_of_polygons"]} \n'
               f'Sum of Widths = {round(stats_dict["sum_of_widths"], 2)}')

        stats_label.config(text=str,
                           justify="left",  # Justify text to the left
                           anchor="w")

def show_coverage():
    """
    covered_area, coverage_percentage = compute_covered_area_with_obstacles(region, obstacles, path, chosen_path_width)
    outlier_area = compute_outlier_area(region, path, chosen_path_width)
    overlap_area = compute_overlap_area(region, obstacles, path, chosen_path_width)

    fig = plot_cpp.plot_coverage(region, path, chosen_path_width, covered_area, outlier_area, overlap_area, obstacles,
                                 sorted_sub_polygons)
    """

def path_planner():
    global current_plot_index, sorting_variable, tsp_iterations, show_coverage_var

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
                    chosen_overlap_distance = float(overlap_value)

                    # Removing collinear vertices
                    removed_col_sub_polygons = []
                    for poly in sub_polygons:
                        removed_col_sub_polygons.append(decomposition.remove_collinear_vertices(poly))
                    sub_polygons = removed_col_sub_polygons

                    total_start_time = time.time()
                    intersections = cpp_path_planning.multi_intersection_planning(sub_polygons, chosen_path_width, chosen_overlap_distance)

                    if sorting_variable.get() == 'DFS':
                        print("DFS")
                        sorting_start_time = time.time()
                        sorted_sub_polygons, sorted_intersections = sorting_dfs_adjacency_graph.solve_dfs(sub_polygons, intersections)
                        sorting_end_time = time.time()
                        total_sorting_time = sorting_end_time - sorting_start_time

                    elif sorting_variable.get() == 'TSP Centroid':
                        print("Centroids")
                        sorting_start_time = time.time()
                        sorted_sub_polygons, sorted_intersections = sorting_tsp_centroid.solve_centroid_tsp(sub_polygons, intersections)
                        sorting_end_time = time.time()
                        total_sorting_time = sorting_end_time - sorting_start_time

                    elif sorting_variable.get() == 'TSP Intra Regional':
                        print("Intra Regional")
                        sorting_start_time = time.time()
                        value = tsp_iterations.get()

                        if value:
                            value = int(value)
                            if value > 0:
                                sorted_sub_polygons, sorted_intersections = sorting_tsp_intra_regional.solve_intra_regional_tsp(sub_polygons, intersections, value)
                            else:
                                sorted_sub_polygons = sub_polygons
                                sorted_intersections = intersections

                        sorting_end_time = time.time()
                        total_sorting_time = sorting_end_time - sorting_start_time
                    else:
                        print("Unordered")
                        sorted_sub_polygons = sub_polygons
                        sorted_intersections = intersections
                        total_sorting_time = 0

                    # Computing path
                    path, transit_flags = cpp_connect_path.connect_path(sorted_sub_polygons, sorted_intersections, region)

                    # Computing plot for path
                    fig_path = plot_cpp.plot_multi_polys_path(region, chosen_path_width, sorted_sub_polygons, path, obstacles, False, transit_flags)

                    # Ending timer and computing total execution time
                    total_end_time = time.time()
                    total_execution_time = total_end_time - total_start_time

                    # Computing data about path
                    stats_dict = compute_path_data(region, sorted_sub_polygons, path, chosen_path_width, obstacles, total_execution_time)
                    stats_dict['total_execution_time'] = total_execution_time
                    stats_dict['sorting_variable'] = sorting_variable.get()
                    stats_dict['sorting_time'] = total_sorting_time
                    stats_dict['path_width'] = chosen_path_width
                    stats_dict['overlap_distance'] = chosen_overlap_distance
                    stats.append(stats_dict)
                    sub_polygons_list.append(None)
                    plots.append(fig_path)

                    # Checking if user wants to see coverage plot
                    if show_coverage_var.get():
                        fig_coverage = plot_cpp.plot_coverage(region, path, chosen_path_width, stats_dict["covered_area"],
                                                              stats_dict["outlying_area"],
                                                              stats_dict["overlapped_area"], obstacles,
                                                              sorted_sub_polygons, transit_flags)
                        plots.append(fig_coverage)
                        sub_polygons_list.append(None)
                        stats.append(stats_dict)

                    current_plot_index = len(plots) - 1
                    update_plot()

                except ValueError:
                    None

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

def save_data():
    global file_path

    if file_path is not None:
        if sub_polygons_list[current_plot_index] is not None:
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
    global canvas, plot_index, stats_label, toolbar, canvas_frame

    canvas_frame = Frame(plot_pane, bg='white')
    canvas_frame.pack(fill='both')

    canvas = FigureCanvasTkAgg(None, master=canvas_frame)
    canvas.draw()
    canvas.get_tk_widget().pack(side='top', fill='both', expand=True)

    # Add a matplotlib navigation toolbar
    toolbar = NavigationToolbar2Tk(canvas, canvas_frame)
    toolbar.update()
    toolbar.pack(side='bottom', fill='x')

    # Frame to contain the buttons
    button_frame = Frame(plot_pane)
    button_frame.pack(fill="x", padx=10, pady=10)

    prev_btn = Button(button_frame, text='Previous', command=prev_plot)
    prev_btn.pack(side="left", anchor='w')

    next_btn = Button(button_frame, text='Next', command=next_plot)
    next_btn.pack(side='right', anchor='e')

    plot_index = Label(button_frame, text=f'{current_plot_index} / {len(plots)}', font=("Arial", 10))
    plot_index.place(relx=0.5, rely=0.5, anchor="center")  # Anchor to center

    Button(button_frame, text='Save Data', command=save_data).place(relx=0.6, rely=0.5, anchor="center")

    stats_label = Label(plot_pane, text='', font=("Arial", 10))
    stats_label.pack(anchor='w')

"""def show_checkbox_state():
    global checkbox_var

    if checkbox_var.get() == 1:
        print("Checkbox is checked")
    else:
        print("Checkbox is unchecked")"""

def toggle_sorting_method():
    global tsp_iterations, sorting_variable
    if sorting_variable.get() == 'TSP Intra Regional':
        tsp_iterations.config(state="normal")
    else:
        tsp_iterations.config(state="disabled")

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
    global label, decomposition_variable, path_width_entry, overlap_distance_entry, sorting_variable, tsp_iterations, show_coverage_var

    Label(options_pane, text='Select File', font=("Arial", 14)).pack(anchor='w')
    Button(options_pane, text="Select File", command=select_file).pack(anchor='w')

    label = Label(options_pane, text='', font=("Arial", 10))
    label.pack(anchor='w')

    Label(options_pane, text='Decomposition Algorithm', font=('Arial, 14')).pack(anchor='w', pady=(15, 0))

    decomposition_variable = StringVar(value='Greedy Recursive')
    rb1 = Radiobutton(options_pane, text='Greedy Recursive', variable=decomposition_variable, value='Greedy Recursive')
    rb2 = Radiobutton(options_pane, text='Sweep Line', variable=decomposition_variable, value='Sweep Line')
    rb3 = Radiobutton(options_pane, text='Combination', variable=decomposition_variable, value='Combination')
    rb1.pack(anchor='w')
    rb2.pack(anchor='w')
    rb3.pack(anchor='w')

    Button(options_pane, text="Decompose", command=decompose).pack(anchor='w')
    Button(options_pane, text='Optimize', command=optimize).pack(anchor='w', pady=(5, 0))

    Label(options_pane, text='Path Width', font=('Arial, 14')).pack(anchor='w', pady=(15, 0))

    path_width_entry = Entry(options_pane, validate="key", validatecommand=(validate_cmd, "%P"))
    path_width_entry.pack(anchor='w')
    path_width_entry.insert(0, '10')

    Label(options_pane, text='Path Overlap Distance', font=('Arial, 14')).pack(anchor='w', pady=(10, 0))
    overlap_distance_entry = Entry(options_pane, validate="key", validatecommand=(validate_cmd, "%P"))
    overlap_distance_entry.pack(anchor='w')
    overlap_distance_entry.insert(0, '0')

    Label(options_pane, text='Sorting Method', font=('Arial, 14')).pack(anchor='w', pady=(15, 0))

    sorting_variable = StringVar(value='Unordered')
    rb4 = Radiobutton(options_pane, text='Unordered', variable=sorting_variable, value='Unordered', command=toggle_sorting_method)
    rb5 = Radiobutton(options_pane, text='DFS', variable=sorting_variable, value='DFS', command=toggle_sorting_method)
    rb6 = Radiobutton(options_pane, text='TSP Centroid', variable=sorting_variable, value='TSP Centroid', command=toggle_sorting_method)
    rb7 = Radiobutton(options_pane, text='TSP Intra Regional', variable=sorting_variable, value='TSP Intra Regional', command=toggle_sorting_method)
    rb4.pack(anchor='w')
    rb5.pack(anchor='w')
    rb6.pack(anchor='w')
    rb7.pack(anchor='w')

    #tsp_iterations = Text(options_pane)
    #tsp_iterations.pack(anchor='w')
    #tsp_iterations.insert(END, '10')
    Label(options_pane, text='Iterations', font=("Arial", 10)).pack(anchor='w')

    tsp_iterations = Entry(options_pane, validate="key", validatecommand=(validate_cmd, "%P"))
    tsp_iterations.pack(anchor='w')
    tsp_iterations.insert(0, '10')
    tsp_iterations.config(state="disabled")

    Label(options_pane, text='Path Planner', font=('Arial, 14')).pack(anchor='w', pady=(15, 0))

    show_coverage_var = IntVar()

    Checkbutton(options_pane, text="Show Coverage Plot", variable=show_coverage_var).pack(anchor='w')
    Button(options_pane, text='Create Path', command=path_planner).pack(anchor='w')

    # Showing coverage plot
    #Button(options_pane, text='Show Coverage', command=show_coverage).pack(anchor='w')

# Initialize main Tkinter window
root = Tk()
root.title('Adaptive Route Planning')

# Register the validation function with Tkinter
validate_cmd = root.register(validate_integer_input)

# Set the window size to the screen size (maximized window)
screen_width = root.winfo_screenwidth()
screen_height = root.winfo_screenheight()
root.geometry(f"{screen_width}x{screen_height}+0+0")

paned_window = PanedWindow(root, orient=HORIZONTAL)
paned_window.pack(fill=BOTH, expand=1)

plot_pane = Frame(paned_window, width=500, height=600)
paned_window.add(plot_pane)

options_pane = Frame(paned_window, width=300, height=600)
paned_window.add(options_pane)

# Set up the plot pane
setup_plot_pane()

# Set up the option pane
setup_option_pane()

# Initialize the plot list and the index
#plots = create_plots()
#current_plot_index = 0

root.mainloop()