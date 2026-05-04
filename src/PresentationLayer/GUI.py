import customtkinter as ctk
from PIL import Image, ImageOps

from src.PresentationLayer.Animation import Animation


class GUI:
    def __init__(self, system):
        self.system = system
        self.storage_objects = system.get_objects()

        # Load in the images used as symbols for the GUI
        self._load_images()

        # Color palette
        self.bg_color = "#495057"  # Stacks (Medium Grey)
        self.box_color = "#6C757D"  # Item Rows (Light Grey)
        self.title_color = "#F8F9FA"  # Headers (Bright White)
        self.text_color = "#E9ECEF"  # Item Text (Soft White)
        self.border_color = "#ADB5BD"  # Borders (Brighter highlight)

        # Create window
        self.root = ctk.CTk(fg_color=self.bg_color)
        self.root.title("Storage Overview")
        w = self.root.winfo_screenwidth()
        h = self.root.winfo_screenheight()
        self.root.geometry(f"{w}x{h}")

        # Labels for stacks
        self.labels = ["Storage_0", "In_Transit", "Storage_1"]

        # Keep references to stack frames so we can update them later
        # This is going to show whether an object is in storage 0, storage 1 or in transit
        self.frames = {}
        self.last_state = {}
        for col, label in enumerate(self.labels):
            stack = self._create_stack(self.root, label, self.box_color)
            if col == 0:
                stack.grid(row=0, column=col, padx=15, pady=15, sticky="nsew")
            elif col == 1:
                stack.grid(row=0, column=col, columnspan=2, padx=15, pady=15, sticky="nsew")
            elif col == 2:
                stack.grid(row=0, column=col + 1, padx=15, pady=15, sticky="nsew")
            self.frames[label] = stack

        # Create the ANimation Canvas
        self.animation = Animation(self.root, self.storage_objects)

        # Make the grid cells expand
        grid_configs = [2, 1, 1, 2]
        row_configs = [51, 49]
        for i in range(4):
            self.root.grid_columnconfigure(i, weight=grid_configs[i])
        for i in range(2):
            self.root.grid_rowconfigure(i, weight=row_configs[i], uniform="row_group")

        # Create the terminal fro anomaly logs
        self._create_anomaly_log()
        # Create the settings button
        self._create_buttons()

        # Start the periodic update loop
        self._update_storage_objects()
        self._update_animation()

        # Start the main GUI loop
        self.root.mainloop()

    def _load_images(self):
        # Load the images
        diff_img = Image.open("resources/Images/different_storage.png").convert("RGBA")
        same_img = Image.open("resources/Images/same_storage.png").convert("RGBA")

        # Apply the transformation
        white_diff = self._make_white(diff_img)
        white_same = self._make_white(same_img)

        # Update your ctkImage references
        self.different_storage_image = ctk.CTkImage(light_image=white_diff, dark_image=white_diff, size=(20, 20))
        self.same_storage_image = ctk.CTkImage(light_image=white_same, dark_image=white_same, size=(20, 20))

    def _create_buttons(self):
        dt_frame = ctk.CTkFrame(self.root, fg_color=self.box_color, corner_radius=10, width=100, height=100)
        dt_frame.pack_propagate(False)

        # ctk.CTkLabel(dt_frame, text="Settings", font=("Arial", 16, "bold"), fg_color="transparent", text_color=self.title_color).pack(pady=(10, 0))

        # time_based_dt_button = ctk.CTkButton(dt_frame, fg_color=self.bg_color, hover_color=self.border_color, text="Time based DT", command=lambda: self.system.stop_system(), width=120, height=35)
        # time_based_dt_button.pack(pady=(10, 10))

        # vision_based_dt_button = ctk.CTkButton(dt_frame, fg_color=self.bg_color, hover_color=self.border_color, text="Vision based DT", command=lambda: self.system.stop_system(), width=120, height=35)
        # vision_based_dt_button.pack(pady=(10, 10))

        # no_dt_button = ctk.CTkButton(dt_frame, fg_color=self.bg_color, hover_color=self.border_color, text="No DT", command=lambda: self.system.stop_system(), width=120, height=35)
        # no_dt_button.pack(pady=(10, 10))

        ctk.CTkLabel(dt_frame, text="Turn on/off system", font=("Arial", 16, "bold"), fg_color="transparent", text_color=self.title_color).pack(pady=(10, 0))

        start_button = ctk.CTkButton(dt_frame, fg_color="green", hover_color=self.border_color, text="Vision based DT", command=lambda: self.system.stop_system(), width=120, height=35)
        start_button.pack(pady=(10, 10))

        exit_button = ctk.CTkButton(dt_frame, fg_color="#880015", hover_color=self.border_color, text="Shut down system", command=lambda: self.system.stop_system(), width=120, height=35)
        exit_button.pack(pady=(10, 10))
        dt_frame.grid(row=1, column=2, sticky="news", padx=15, pady=15)

    def _create_anomaly_log(self):
        # Change CTkFrame to CTkScrollableFrame
        self.log_frame = ctk.CTkScrollableFrame(
            self.root,
            fg_color=self.box_color,
            corner_radius=10,
            label_fg_color=self.box_color,
            label_text="Logs",  # Optional: Adds a built-in header
            label_font=("Arial", 16, "bold"),
            label_text_color=self.title_color,
        )

        # Grid placement remains the same
        self.log_frame.grid(row=1, column=3, sticky="news", padx=15, pady=15)

    # Function to "color" a black icon to white
    def _make_white(self, img):
        r, g, b, a = img.split()
        # Invert the RGB channels (Black becomes White) but keep the original Alpha (transparency)
        return Image.merge("RGBA", (ImageOps.invert(r), ImageOps.invert(g), ImageOps.invert(b), a))

    # Create the stack for on of the storages or in transit
    def _create_stack(self, parent, title, color):
        stack_frame = ctk.CTkFrame(parent, fg_color=color, corner_radius=10)
        stack_frame.pack_propagate(False)

        # Convert the titles from underscore tol space
        new_title = ""
        if title == "In_Transit":
            new_title = "In Transit"
        else:
            new_title = f"{title[:-2]} {title[-1]}"

        ctk.CTkLabel(stack_frame, text=new_title, font=("Arial", 16, "bold"), fg_color=color, text_color=self.title_color).pack(pady=(10, 10))

        # Add a container for the object buttons
        stack_frame.object_container = ctk.CTkFrame(stack_frame, fg_color="transparent")
        stack_frame.object_container.pack(fill="both", expand=True, padx=5, pady=(0, 10))

        # This fills in the objects in the stack
        self._populate_stack(stack_frame, title)
        return stack_frame

    # This fills in the objects in the stack
    def _populate_stack(self, stack_frame, title):
        # Clear previous widgets
        for widget in stack_frame.object_container.winfo_children():
            widget.destroy()

        # Add current objects
        for obj in self.storage_objects:
            if obj.position == title:
                row_frame = ctk.CTkFrame(stack_frame.object_container, fg_color=self.box_color, border_color=self.border_color, border_width=2, corner_radius=5)
                row_frame.pack(fill="x", padx=10, pady=6)

                row_frame.grid_columnconfigure(0, weight=70)
                row_frame.grid_columnconfigure(1, weight=0)
                row_frame.grid_columnconfigure(2, weight=15)
                row_frame.grid_columnconfigure(3, weight=0)
                row_frame.grid_columnconfigure(4, weight=15)

                ctk.CTkLabel(row_frame, text=obj.name, font=("Arial", 16, "bold"), text_color=self.text_color).grid(row=0, column=0, pady=6)

                for j in range(1, 4, 2):
                    separator = ctk.CTkFrame(row_frame, fg_color=self.border_color, height=35, width=2)
                    separator.grid(row=0, column=j, pady=5)

                for j in range(2, 5, 2):
                    destination = "Storage_0" if obj.position == "Storage_1" and j == 4 else "Storage_1" if obj.position == "Storage_0" and j == 4 else obj.position
                    params = [obj.name, destination]
                    btn = ctk.CTkButton(
                        row_frame,
                        fg_color="transparent",
                        hover_color=self.border_color,
                        text="",
                        image=self.same_storage_image if j == 2 else self.different_storage_image,
                        width=20,
                        height=20,
                        command=lambda p=params: self._clicked(p),
                    )
                    btn.grid(row=0, column=j)

    # Add log message to the log terminal
    def _add_log(self, anomaly_log_object):
        time, actor, anomaly_text = anomaly_log_object
        if actor == 1 or actor == 0:
            actor = f"Robot {actor}"
        text = f"[{time}] {actor} - {anomaly_text}"

        # Create the label inside the scrollable frame
        label = ctk.CTkLabel(self.log_frame, text=text, anchor="w", justify="left", text_color=self.text_color)
        label.pack(fill="x", padx=10, pady=2)

        # Wrap text based on the width of the scrollable area
        # Note: Using 30-40 offset to account for the scrollbar width
        self.log_frame.update_idletasks()
        label.configure(wraplength=self.log_frame.winfo_width() - 40)

        children = self.log_frame.winfo_children()
        if len(children) > 100:  # Keep last 100 logs
            children[0].destroy()

    # Update the storage objects based on information from system
    def _update_storage_objects(self):
        # Refresh object list
        self.storage_objects = self.system.get_objects()

        # Update each stack
        for label in self.labels:
            # 1. Filter objects for this specific stack
            stack_objects = [obj for obj in self.storage_objects if obj.position == label]

            # 2. Create a "signature" representing the current state.
            #    We use a tuple of object names to detect if anything changed.
            #    (If you have other changing attributes like color, add them here too)
            current_signature = tuple(obj.name for obj in stack_objects)

            # 3. Compare with the last known state
            if self.last_state.get(label) != current_signature:
                # Data changed! Update the UI and save the new state
                self._populate_stack(self.frames[label], label)
                self.last_state[label] = current_signature

        # Call again after 500 ms
        self.root.after(500, self._update_storage_objects)

    # Update the Animation with DT data every 100 ms
    def _update_animation(self):
        animation_info, anomaly_log_objects = self.system.get_info_dt()
        for anomaly_log_object in anomaly_log_objects:
            self._add_log(anomaly_log_object)
        self.animation.set_info_dt(animation_info)
        self.root.after(100, self._update_animation)

    # Call move_object function in system when a button has been clicked in
    def _clicked(self, params):
        self.system.move_object(*params)
