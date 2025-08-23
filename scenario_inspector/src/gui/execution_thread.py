"""
Background thread for executing inspection programs without blocking the UI.
"""
import logging
import threading
from inspector.scenario_engine import ScenarioEngine
from concurrent.futures import ThreadPoolExecutor, as_completed
from PyQt5.QtCore import QThread, pyqtSignal


class ProgramExecutionThread(QThread):
    """Thread for executing inspection programs without blocking the UI.

    Signals:
        progress_updated (str): Emitted with human-friendly progress messages.
        execution_finished (bool, str, object): Emitted when execution ends with
            success flag, output folder path, and inspection date (datetime or None).
        step_completed (str): Emitted when a single step completes with the step name.
        progress_percentage (int): Emitted to update a determinate progress bar.
        pause_signal (str): Emitted when a pause is requested, carrying stage name.

    Attributes:
        program_data (dict): Parsed YAML program data to execute.
        piece_info (dict): Piece metadata from the GUI (name, ref).
        config_manager: App configuration accessor.
        scenario_engine: Orchestrator handling actual scenario execution.
        inspection_folder (str|None): Final folder where results were written.
        inspection_date (datetime|None): Timestamp of the inspection start/end.
    """

    progress_updated = pyqtSignal(str)  # Progress message
    execution_finished = pyqtSignal(bool, str, object)  # Success status, folder path, date
    step_completed = pyqtSignal(str)  # Step name
    progress_percentage = pyqtSignal(int)  # Progress percent for progress bar
    pause_signal = pyqtSignal(str)  # Emitted when a pause is requested (stage_name)

    def __init__(self, program_data, piece_info, config_manager):
        super().__init__()
        self.program_data = program_data
        self.piece_info = piece_info
        self.config_manager = config_manager
        self.logger = logging.getLogger(__name__)

        self._pause_event = threading.Event()
        self._pause_event.set()  # Start unpaused

        self.scenario_engine = ScenarioEngine(self.config_manager, self._emit_progress)
        self.scenario_engine.pause_callback = self._on_pause
        self.scenario_engine.progress_callback_percentage = self._on_progress_percentage

        # Runtime fields
        self.inspection_folder = None
        self.inspection_date = None
        self.execution_results = []

    def _on_progress_percentage(self, percent):
        """Bridge ScenarioEngine percentage to the GUI via a signal."""
        self.progress_percentage.emit(percent)

    def _on_pause(self, stage_name):
        """Handle a stage-level pause by blocking until user resumes.

        Args:
            stage_name: The name of the stage that requested a pause.
        """
        self._pause_event.clear()
        self.pause_signal.emit(stage_name)
        self._pause_event.wait()  # Block until resume_from_pause is called

    def resume_from_pause(self):
        """Resume a paused execution (typically called from the GUI)."""
        self._pause_event.set()

    def _emit_progress(self, message: str):
        """Forward a textual progress update to the GUI."""
        self.progress_updated.emit(message)

    def run(self):
        """Main execution entry point of the worker thread.

        Delegates execution to ScenarioEngine and emits completion signals
        along with the inspection output location and date.
        """
        try:
            success = self.scenario_engine.execute_scenario_from_gui(self.program_data, self.piece_info)
            self.inspection_folder = self.scenario_engine.current_inspection_folder
            self.inspection_date = self.scenario_engine.current_inspection_date

            if success:
                self.progress_percentage.emit(100)
                self.execution_finished.emit(True, self.inspection_folder or "", self.inspection_date)
            else:
                self.execution_finished.emit(False, self.inspection_folder or "", self.inspection_date)
        except Exception as e:
            self.logger.error("Program execution error: %s", e)
            self.progress_updated.emit(f"Execution failed: {str(e)}")
            self.execution_finished.emit(False, self.inspection_folder or "", self.inspection_date)

    # Keep parallel/sequential helpers for future if needed. They rely on a SystemManager
    # and can be progressively reintroduced if required by ScenarioEngine behavior.
