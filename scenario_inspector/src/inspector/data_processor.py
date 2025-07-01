from datetime import datetime
import os
import json

class DataProcessor:
    def __init__(self, output_dir):
        self.output_dir = output_dir

    def create_date_stamped_folder(self):
        date_str = datetime.now().strftime("%Y-%m-%d")
        folder_path = os.path.join(self.output_dir, date_str)
        os.makedirs(folder_path, exist_ok=True)
        return folder_path

    def save_inspection_data(self, data, folder_path):
        file_name = f"inspection_data_{datetime.now().strftime('%H-%M-%S')}.json"
        file_path = os.path.join(folder_path, file_name)
        with open(file_path, 'w') as f:
            json.dump(data, f, indent=4)

    def prepare_data_for_db(self, data):
        # Transform data as needed for database insertion
        processed_data = {
            "inspection_id": data.get("id"),
            "timestamp": data.get("timestamp"),
            "results": data.get("results"),
        }
        return processed_data