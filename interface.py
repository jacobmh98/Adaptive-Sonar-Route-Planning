import copy
from tkinter import *
from tkinter import filedialog

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

import connecting_path
import coverage_plots
import intra_regional_tsp
import multi_poly_planning
import traveling_salesman_variation
from decomposition import sum_of_widths
from load_data import get_region, generate_new_data
from obstacles import plot_obstacles, decompose_sweep_line, merge_filtered_sub_polygons, find_bounding_polygons
from path_comparison_functions import compute_path_data

file_path = None
plots = []
current_plot_index = 0
region = None
obstacles = None
sub_polygons_list = []
stats = []

def select_file():
    global file_path, region, obstacles, plots

    file_path = filedialog.askopenfilename(
        title="Select a File",
        filetypes=(("JSON files", "*.json"), ("All files", "*.*"))
    )

    if file_path:
        label.config(text=f"{file_path}")

    region, obstacles = get_region(file_path)
    fig = plot_obstacles([region], obstacles, False)
    plots.append(fig)
    sub_polygons_list.append(None)
    stats.append(None)
    update_plot()

def decompose():
    global current_plot_index

    if decomposition_variable.get() == 'Greedy Recursive':
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

    if decomposition_variable.get() == 'Sweep Line':
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

    if decomposition_variable.get() == 'Combination':
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

    current_plot_index = len(plots) - 1
    update_plot()

def update_plot():
    global current_plot_index, canvas
    canvas.figure = plots[current_plot_index]

    canvas.draw()
    update_stats()

def next_plot():
    """ Function to show the next plot """
    global current_plot_index
    current_plot_index = (current_plot_index + 1) % len(plots)
    update_plot()

def prev_plot():
    """ Function to show the previous plot """
    global current_plot_index
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
               f'Coverage Percentage = {round(stats_dict["coverage_percentage"], 2)} % \n'
               f'Covered Area = {round(stats_dict["covered_area"], 2)} m^2 \n'
               f'Distance = {round(stats_dict["distance"], 2)} m \n'
               f'Total Turns = {stats_dict["total_turns"]} \n'
               f'Hard Turns (<45) = {stats_dict["hard_turns"]} \n'
               f'Medium Turns (45-90) = {stats_dict["medium_turns"]} \n'
               f'Soft Turns (>90) = {stats_dict["soft_turns"]}')

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

def path_planner():
    global current_plot_index, sorting_variable
    sub_polygons = sub_polygons_list[current_plot_index]

    if sub_polygons is not None:
        value = path_width_entry.get()
        print(f'{value=}')
        if value:  # Check if it's not empty
            try:
                # Convert to integer
                global path_width
                path_width = float(value)

                if sorting_variable.get() == 'DFS':
                    print("DFS")
                    sorted_sub_polygons = multi_poly_planning.sort_sub_polygons_using_dfs(sub_polygons)
                elif sorting_variable.get() == 'TSP Centroid':
                    print("Centroids")
                    sorted_sub_polygons = traveling_salesman_variation.solve_centroid_tsp(sub_polygons)
                elif sorting_variable.get() == 'Intra Regional TSP':
                    print("Intra Regional")
                    intersections = multi_poly_planning.multi_intersection_planning(sub_polygons, path_width)
                    sorted_sub_polygons = intra_regional_tsp.solve_intra_regional_tsp(sub_polygons, intersections)
                else:
                    print("Unordered")
                    sorted_sub_polygons = sub_polygons

                intersections = multi_poly_planning.multi_intersection_planning(sorted_sub_polygons, path_width)
                path = connecting_path.connect_path(sorted_sub_polygons, intersections, region)
                fig = coverage_plots.multi_poly_plot(region, path_width, sorted_sub_polygons, path)

                stats_dict = compute_path_data(region, path, 0)
                stats_dict['sorting_variable'] = sorting_variable.get()
                stats.append(stats_dict)
                plots.append(fig)
                sub_polygons_list.append(None)
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

def setup_plot_pane():
    """ Set up the initial canvas with the first plot """
    global canvas, plot_index, stats_label

    canvas = FigureCanvasTkAgg(None, master=plot_pane)
    canvas.draw()
    canvas.get_tk_widget().pack()

    # Frame to contain the buttons
    button_frame = Frame(plot_pane)
    button_frame.pack(fill="x", padx=10, pady=10)

    prev_btn = Button(button_frame, text='Previous', command=prev_plot)
    prev_btn.pack(side="left", anchor='w')

    next_btn = Button(button_frame, text='Next', command=next_plot)
    next_btn.pack(side='right', anchor='e')

    plot_index = Label(button_frame, text=f'{current_plot_index} / {len(plots)}', font=("Arial", 10))
    plot_index.place(relx=0.5, rely=0.5, anchor="center")  # Anchor to center

    stats_label = Label(plot_pane, text='', font=("Arial", 10))
    stats_label.pack(anchor='w')

def setup_option_pane():
    """ Creating the options pane """
    global label, decomposition_variable, path_width_entry, sorting_variable

    Label(options_pane, text='Select File', font=("Arial", 14)).pack(anchor='w')
    Button(options_pane, text="Select File", command=select_file).pack(anchor='w')

    label = Label(options_pane, text='', font=("Arial", 10))
    label.pack(anchor='w')

    Label(options_pane, text='Decomposition Algorithm', font=('Arial, 14')).pack(anchor='w')

    decomposition_variable = StringVar(value='Greedy Recursive')
    rb1 = Radiobutton(options_pane, text='Greedy Recursive', variable=decomposition_variable, value='Greedy Recursive')
    rb2 = Radiobutton(options_pane, text='Sweep Line', variable=decomposition_variable, value='Sweep Line')
    rb3 = Radiobutton(options_pane, text='Combination', variable=decomposition_variable, value='Combination')
    rb1.pack(anchor='w')
    rb2.pack(anchor='w')
    rb3.pack(anchor='w')

    Button(options_pane, text="Decompose", command=decompose).pack(anchor='w')

    Label(options_pane, text='Path Width', font=('Arial, 14')).pack(anchor='w', pady=(25, 0))

    path_width_entry = Entry(options_pane, validate="key", validatecommand=(validate_cmd, "%P"))
    path_width_entry.pack(anchor='w')
    path_width_entry.insert(0, '10')

    Label(options_pane, text='Sorting Method', font=('Arial, 14')).pack(anchor='w', pady=(25, 0))
    sorting_variable = StringVar(value='TSP Centroid')
    rb4 = Radiobutton(options_pane, text='Unordered', variable=sorting_variable, value='Unordered')
    rb5 = Radiobutton(options_pane, text='DFS', variable=sorting_variable, value='DFS')
    rb6 = Radiobutton(options_pane, text='TSP Centroid', variable=sorting_variable, value='TSP Centroid')
    rb7 = Radiobutton(options_pane, text='Intra Regional TSP', variable=sorting_variable, value='Intra Regional TSP')
    rb4.pack(anchor='w')
    rb5.pack(anchor='w')
    rb6.pack(anchor='w')
    rb7.pack(anchor='w')

    Label(options_pane, text='Path Planner', font=('Arial, 14')).pack(anchor='w', pady=(25, 0))
    Button(options_pane, text='Create Path', command=path_planner).pack(anchor='w')

# Initialize main Tkinter window
root = Tk()
root.title('Adaptive Route Planning')

# Register the validation function with Tkinter
validate_cmd = root.register(validate_integer_input)

# Get the screen width and height
screen_width = root.winfo_screenwidth()
screen_height = root.winfo_screenheight()

# Set the geometry to fill the screen
root.geometry("%dx%d" % (screen_width, screen_height))

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