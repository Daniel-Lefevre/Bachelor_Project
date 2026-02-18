import customtkinter as ctk
from PIL import Image, ImageOps

from src.PresentationLayer.Animation import *


class GUI:
    def __init__(self, system):
        self.system = system
        self.storageObjects = system.getObjects()

        self.loadImages()

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
        self.frames = {}
        self.last_state = {}

        for col, label in enumerate(self.labels):
            stack = self.create_stack(self.root, label, self.box_color)
            if col == 0:
                stack.grid(row=0, column=col, padx=15, pady=15, sticky="nsew")
            elif col == 1:
                stack.grid(row=0, column=col, columnspan=2, padx=15, pady=15, sticky="nsew")
            elif col == 2:
                stack.grid(row=0, column=col + 1, padx=15, pady=15, sticky="nsew")
            self.frames[label] = stack

        self.Animation = Animation(self.root, self.storageObjects)

        # Make the grid cells expand
        gridConfigs = [2, 1, 1, 2]
        for i in range(4):
            self.root.grid_columnconfigure(i, weight=gridConfigs[i])

        for i in range(2):
            self.root.grid_rowconfigure(i, weight=1)

        self.createAnomalyLog()
        self.createButtons()

        # Start the periodic update loop
        self.updateStorageObjects()

        # Start the main GUI loop
        self.root.mainloop()

    def loadImages(self):
        # Load the images
        diff_img = Image.open("resources/Images/different_storage.png").convert("RGBA")
        same_img = Image.open("resources/Images/same_storage.png").convert("RGBA")

        # Apply the transformation
        white_diff = self.make_white(diff_img)
        white_same = self.make_white(same_img)

        # Update your ctkImage references
        self.different_storage_image = ctk.CTkImage(light_image=white_diff, dark_image=white_diff, size=(20, 20))
        self.same_storage_image = ctk.CTkImage(light_image=white_same, dark_image=white_same, size=(20, 20))

    def createButtons(self):
        DT_frame = ctk.CTkFrame(self.root, fg_color=self.box_color, corner_radius=10, width=100, height=100)
        DT_frame.pack_propagate(False)

        ctk.CTkLabel(DT_frame, text="Logs", font=("Arial", 16, "bold"), fg_color="transparent", text_color=self.title_color).pack(pady=(10, 0))

        timeBasedDTButton = ctk.CTkButton(DT_frame, fg_color=self.bg_color, hover_color=self.border_color, text="Time based DT", command=lambda: self.system.stopSystem(), width=120, height=35)
        timeBasedDTButton.pack(pady=(10, 10))

        visionBasedDTButton = ctk.CTkButton(DT_frame, fg_color=self.bg_color, hover_color=self.border_color, text="Vision based DT", command=lambda: self.system.stopSystem(), width=120, height=35)
        visionBasedDTButton.pack(pady=(10, 10))

        noDTButton = ctk.CTkButton(DT_frame, fg_color=self.bg_color, hover_color=self.border_color, text="No DT", command=lambda: self.system.stopSystem(), width=120, height=35)
        noDTButton.pack(pady=(10, 10))

        ctk.CTkLabel(DT_frame, text="Turn on/off system", font=("Arial", 16, "bold"), fg_color="transparent", text_color=self.title_color).pack()

        startButton = ctk.CTkButton(DT_frame, fg_color="green", hover_color=self.border_color, text="Vision based DT", command=lambda: self.system.stopSystem(), width=120, height=35)
        startButton.pack(pady=(10, 10))

        exitButton = ctk.CTkButton(DT_frame, fg_color="#880015", hover_color=self.border_color, text="Shut down system", command=lambda: self.system.stopSystem(), width=120, height=35)
        exitButton.pack(pady=(10, 10))
        DT_frame.grid(row=1, column=2, sticky="news", padx=15, pady=15)

    def createAnomalyLog(self):
        log_frame = ctk.CTkFrame(self.root, fg_color=self.box_color, corner_radius=10)
        # log_frame.pack_propagate(False)

        ctk.CTkLabel(log_frame, text="Logs", font=("Arial", 16, "bold"), fg_color="transparent", text_color=self.title_color).pack(pady=(10, 10))
        log_frame.grid(row=1, column=3, sticky="news", padx=15, pady=15)

    # Function to "color" a black icon to white
    def make_white(self, img):
        r, g, b, a = img.split()
        # Invert the RGB channels (Black becomes White) but keep the original Alpha (transparency)
        return Image.merge("RGBA", (ImageOps.invert(r), ImageOps.invert(g), ImageOps.invert(b), a))

    def create_stack(self, parent, title, color):
        stack_frame = ctk.CTkFrame(parent, fg_color=color, corner_radius=10)
        stack_frame.pack_propagate(False)

        newTitle = ""
        if title == "In_Transit":
            newTitle = "In Transit"
        else:
            newTitle = f"{title[:-2]} {title[-1]}"

        ctk.CTkLabel(stack_frame, text=newTitle, font=("Arial", 16, "bold"), fg_color=color, text_color=self.title_color).pack(pady=(10, 10))

        # Add a container for the object buttons
        stack_frame.object_container = ctk.CTkFrame(stack_frame, fg_color="transparent")
        stack_frame.object_container.pack(fill="both", expand=True, padx=5, pady=(0, 10))

        self.populate_stack(stack_frame, title)
        return stack_frame

    def populate_stack(self, stack_frame, title):
        # Clear previous widgets
        for widget in stack_frame.object_container.winfo_children():
            widget.destroy()

        # Add current objects
        for obj in self.storageObjects:
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
                        command=lambda p=params: self.clicked(p),
                    )
                    btn.grid(row=0, column=j)

    def updateStorageObjects(self):
        # Refresh object list
        self.storageObjects = self.system.getObjects()

        # Update each stack
        for label in self.labels:
            # 1. Filter objects for this specific stack
            stack_objects = [obj for obj in self.storageObjects if obj.position == label]

            # 2. Create a "signature" representing the current state.
            #    We use a tuple of object names to detect if anything changed.
            #    (If you have other changing attributes like color, add them here too)
            current_signature = tuple(obj.name for obj in stack_objects)

            # 3. Compare with the last known state
            if self.last_state.get(label) != current_signature:
                # Data changed! Update the UI and save the new state
                self.populate_stack(self.frames[label], label)
                self.last_state[label] = current_signature

        # Call again after 500ms
        self.root.after(500, self.updateStorageObjects)

    def clicked(self, params):
        # self.system.move_object(*params)
        pass
