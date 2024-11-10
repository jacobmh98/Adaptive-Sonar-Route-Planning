import copy
import tkinter
from tkinter import *
from tkinter import filedialog
from global_variables import *
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

import connecting_path
import coverage_plots
import multi_poly_planning
from load_data import get_region, generate_new_data
from obstacles import plot_obstacles, decompose_sweep_line, merge_filtered_sub_polygons, find_bounding_polygons

file_path = None
plots = []
current_plot_index = 0
region = None
obstacles = None
sub_polygons_list = []

def select_file():
    global file_path
    file_path = filedialog.askopenfilename(
        title="Select a File",
        filetypes=(("JSON files", "*.json"), ("All files", "*.*"))
    )


    if file_path:
        label.config(text=f"{file_path}")
    global region, obstacles, plots

    region, obstacles = get_region(file_path)
    fig = plot_obstacles([region], obstacles, False)
    plots.append(fig)
    update_plot()
    sub_polygons_list.append(None)

def decompose():
    global current_plot_index

    if var.get() == 'Greedy Recursive':
        # Decompose the region without considering obstacles
        sub_polygons = generate_new_data(copy.deepcopy(region))
        sub_polygons_list.append(sub_polygons)
        fig = plot_obstacles(sub_polygons, obstacles, False)

        plots.append(fig)

    if var.get() == 'Sweep Line':
        # Decompose the region without considering obstacles
        sub_polygons_sweep_line = decompose_sweep_line(copy.deepcopy(region), copy.deepcopy(obstacles))
        sub_polygons_list.append(sub_polygons_sweep_line)
        fig = plot_obstacles(sub_polygons_sweep_line, obstacles, False)
        plots.append(fig)

    if var.get() == 'Combination':
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
        #combined_algorithms(region, obstacles)

    current_plot_index = len(plots) - 1
    update_plot()

def update_plot():
    global current_plot_index
    canvas.figure = plots[current_plot_index]
    canvas.draw()
    update_text()

# Function to show the next plot
def next_plot():
    global current_plot_index
    current_plot_index = (current_plot_index + 1) % len(plots)
    update_plot()

# Function to show the previous plot
def prev_plot():
    global current_plot_index
    current_plot_index = (current_plot_index - 1) % len(plots)
    update_plot()

def update_text():
    text_field.config(text=f"{current_plot_index + 1} / {len(plots)}")

def path_planner():
    global current_plot_index
    sub_polygons = sub_polygons_list[current_plot_index]

    if sub_polygons is not None:

        intersections = multi_poly_planning.multi_intersection_planning(sub_polygons, path_width)
        path = connecting_path.connect_path(sub_polygons, intersections, region)
        fig = coverage_plots.multi_poly_plot(region, path_width, sub_polygons, path)
        plots.append(fig)

        sub_polygons_list.append(None)
        current_plot_index = len(plots) - 1
        update_plot()

# Initialize main Tkinter window
root = Tk()
root.title('Adaptive Route Planning')
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

# Creating the options pane
Label(options_pane, text='Select File', font=("Arial", 14)).pack(anchor='w')
Button(options_pane, text="Select File", command=select_file).pack(anchor='w')

label = Label(options_pane, text='', font=("Arial", 10))
label.pack(anchor='w')

Label(options_pane, text='Decomposition Algorithm', font=('Arial, 14')).pack(anchor='w')

var = StringVar(value='Greedy Recursive')
rb1 = Radiobutton(options_pane, text='Greedy Recursive', variable=var, value='Greedy Recursive')
rb2 = Radiobutton(options_pane, text='Sweep Line', variable=var, value='Sweep Line')
rb3 = Radiobutton(options_pane, text='Combination', variable=var, value='Combination')
rb1.pack(anchor='w')
rb2.pack(anchor='w')
rb3.pack(anchor='w')

Button(options_pane, text="Decompose", command=decompose).pack(anchor='w')

Label(options_pane, text='Path Planner', font=('Arial, 14')).pack(anchor='w', pady=(25, 0))
Button(options_pane, text='Create Path', command=path_planner).pack(anchor='w')


# Set up the initial canvas with the first plot
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

text_field = Label(button_frame, text=f'{current_plot_index} / {len(plots)}', font=("Arial", 10))
text_field.place(relx=0.5, rely=0.5, anchor="center")  # Anchor to center



# Initialize the plot list and the index
#plots = create_plots()
#current_plot_index = 0


root.mainloop()