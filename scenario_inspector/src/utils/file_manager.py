"""
File manager utilities for organizing output data
"""
import os
import shutil
from datetime import datetime
import logging

class FileManager:
    """Manages file operations and output directory structure"""

    def __init__(self, base_output_dir: str):
        self.base_output_dir = base_output_dir
        self.logger = logging.getLogger(__name__)

        # Ensure base output directory exists
        os.makedirs(base_output_dir, exist_ok=True)

    def create_inspection_folder(self, program_name: str, piece_name: str,
                               ref_piece: str) -> str:
        """Create organized folder structure for inspection data

        Creates daily inspection folders with the following structure:
        - output/YYYY-MM-DD/                          (Daily folder for all inspections)
        - output/YYYY-MM-DD/inspection_{PROGRAM}_YYYYMMDD_HHMMSS/  (Individual inspection)

        Examples:
        - output/2025-01-15/inspection_camera_photo_20250115_143025/
        - output/2025-01-15/inspection_wing_scan_20250115_151030/

        Args:
            program_name: Name of the YAML scenario program
            piece_name: Name of the piece being inspected (logged for reference)
            ref_piece: Reference number of the piece (logged for reference)

        Returns:
            str: Path to the created inspection folder
        """
        now = datetime.now()
        date_str = now.strftime("%Y-%m-%d")  # Daily folder format: 2025-01-15
        time_str = now.strftime("%Y%m%d_%H%M%S")  # New timestamp format: 20250115_143025

        # Create date folder for today's inspections
        date_folder = os.path.join(self.base_output_dir, date_str)
        os.makedirs(date_folder, exist_ok=True)

        # Create inspection-specific folder with new naming convention
        # Format: inspection_{PROGRAM}_YYYYMMDD_HHMMSS
        folder_name = f"inspection_{program_name}_{time_str}"
        # Clean folder name (remove invalid characters)
        folder_name = self._clean_filename(folder_name)

        inspection_folder = os.path.join(date_folder, folder_name)
        os.makedirs(inspection_folder, exist_ok=True)

        self.logger.info("Created inspection folder: %s", inspection_folder)
        self.logger.info("Daily folder structure: %s", date_folder)
        self.logger.info("Program: %s, Piece: %s, Reference: %s", program_name, piece_name, ref_piece)

        return inspection_folder

    def organize_step_data(self, inspection_folder: str, step_name: str,
                          step_results: dict) -> str:
        """Organize data from a step into the inspection folder"""
        step_folder = os.path.join(inspection_folder, step_name)
        os.makedirs(step_folder, exist_ok=True)

        # Move any files created by the step
        if step_results.get('file_saved') and step_results.get('output_path'):
            source_path = step_results['output_path']
            if os.path.exists(source_path):
                filename = os.path.basename(source_path)
                dest_path = os.path.join(step_folder, filename)
                try:
                    shutil.move(source_path, dest_path)
                    self.logger.info("Moved %s to %s", filename, step_folder)
                    return dest_path
                except Exception as e:
                    self.logger.error("Failed to move file %s: %s", filename, e)

        return step_folder

    def create_inspection_report(self, inspection_folder: str,
                               inspection_data: dict) -> str:
        """Create inspection report file"""
        report_path = os.path.join(inspection_folder, "inspection_report.txt")
        self.logger.info("Creating inspection report at: %s", report_path)
        try:
            with open(report_path, 'w', encoding='utf-8') as f:
                f.write("AUAS INSPECTION REPORT\n")
                f.write("=" * 50 + "\n\n")

                # Basic information
                f.write(f"Program: {inspection_data.get('program_name', 'Unknown')}\n")
                f.write(f"Piece: {inspection_data.get('piece_name', 'Unknown')}\n")
                f.write(f"Reference: {inspection_data.get('ref_piece', 'Unknown')}\n")
                f.write(f"Inspection Date: {inspection_data.get('inspection_date', 'Unknown')}\n")
                f.write(f"Inspector: {inspection_data.get('inspector', 'Unknown')}\n\n")

                # Step results
                f.write("STEP RESULTS:\n")
                f.write("-" * 30 + "\n")

                steps = inspection_data.get('steps', [])
                for i, step in enumerate(steps, 1):
                    f.write(f"\nStep {i}: {step.get('name', 'Unknown')}\n")
                    f.write(f"  System: {step.get('system', 'Unknown')}\n")
                    f.write(f"  Status: {'SUCCESS' if step.get('success') else 'FAILED'}\n")
                    if step.get('duration'):
                        f.write(f"  Duration: {step.get('duration'):.2f} seconds\n")
                    if step.get('error'):
                        f.write(f"  Error: {step.get('error')}\n")

                # Summary
                f.write("\n\nINSPECTION SUMMARY:\n")
                f.write("-" * 30 + "\n")
                total_steps = len(steps)
                successful_steps = sum(1 for step in steps if step.get('success'))
                f.write(f"Total Steps: {total_steps}\n")
                f.write(f"Successful: {successful_steps}\n")
                f.write(f"Failed: {total_steps - successful_steps}\n")
                f.write(f"Success Rate: {(successful_steps/total_steps*100):.1f}%\n" if total_steps > 0 else "Success Rate: 0%\n")

            self.logger.info("Created inspection report: %s", report_path)
            return report_path

        except Exception as e:
            self.logger.error("Failed to create inspection report: %s", e)
            return ""

    def _clean_filename(self, filename: str) -> str:
        """Clean filename by removing invalid characters"""
        invalid_chars = '<>:"/\\|?*'
        for char in invalid_chars:
            filename = filename.replace(char, '_')
        return filename
