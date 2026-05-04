import os


def fix_by_looping(full_folder_path, max_expected_number):
    counter = 1

    # Loop from 1 up to 173
    for i in range(1, max_expected_number + 1):
        current_filename = f"{i}.jpg"
        current_filepath = os.path.join(full_folder_path, current_filename)

        # If the image exists, process it
        if os.path.exists(current_filepath):
            new_filename = f"{counter}.jpg"
            new_filepath = os.path.join(full_folder_path, new_filename)

            # Only rename if the numbers are different
            if current_filepath != new_filepath:
                os.rename(current_filepath, new_filepath)
                print(f"Renamed: '{current_filename}' -> '{new_filename}'")

            # Increment the counter
            counter += 1

    print(f"\nDone! You now have a solid sequence from 1.jpg to {counter - 1}.jpg.")


if __name__ == "__main__":
    # 1. The name of the folder next to your script
    FOLDER_NAME = "Validation_Data"

    objects = ["Blue_Circle", "Blue_Square", "Green_Circle", "Green_Square", "No_Object", "Red_Circle", "Red_Square", "Unidentified_Object"]

    MAX_EXPECTED = 210

    for object in objects:
        # 2. Get the exact directory where this Python script lives
        script_dir = os.path.dirname(os.path.abspath(__file__))

        # 3. Build the full, absolute path to the Training_Data folder
        target_folder_path = os.path.join(script_dir, FOLDER_NAME, object)

        # 4. Check if it exists using the full path
        if os.path.exists(target_folder_path):
            fix_by_looping(target_folder_path, MAX_EXPECTED)
        else:
            print(f"Error: The folder '{target_folder_path}' does not exist.")
