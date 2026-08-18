# Assuming you have not changed the general structure of the template no modification is needed in this file.
from datetime import datetime
import importlib
import os
import traceback

import adsk.core
import adsk.fusion

from . import commands
from . import helix
from .lib import fusionAddInUtils as futil


WORKING_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
LOG_PATH = os.path.join(WORKING_DIRECTORY, 'log.log')
EXPORT_DIRECTORY = os.path.join(WORKING_DIRECTORY, 'exports')
STEP_FILENAME = 'AnnularFluid_HelixBaffle_Watertight.step'
EXPECTED_BODY_NAMES = (
    'Annular_Fluid_Watertight',
)


def _initialize_log() -> None:
    """Replace the previous log at the beginning of every add-in run."""
    timestamp = datetime.now().astimezone().isoformat(timespec='seconds')
    try:
        with open(LOG_PATH, 'w', encoding='utf-8') as log_file:
            log_file.write(f'[{timestamp}] CodexFusion run started.\n')
    except OSError as error:
        # Logging must never hide the original modeling/export failure.
        adsk.core.Application.log(f'Could not initialize {LOG_PATH}: {error}')


def _write_log(message: str) -> None:
    """Append a timestamped message to the add-in working-directory log."""
    timestamp = datetime.now().astimezone().isoformat(timespec='seconds')
    try:
        with open(LOG_PATH, 'a', encoding='utf-8') as log_file:
            log_file.write(f'[{timestamp}] {message.rstrip()}\n')
    except OSError as error:
        # Logging must never hide the original modeling/export failure.
        adsk.core.Application.log(f'Could not write {LOG_PATH}: {error}')


def _describe_body(body: adsk.fusion.BRepBody) -> str:
    """Describe a body without requesting volume from a sheet body."""
    if not body:
        return '<missing body>'
    if body.isSolid:
        return f'{body.name} (solid, {body.volume:.6g} cm^3)'
    return f'{body.name} (sheet, {body.faces.count} face(s))'


def _validate_step_file(step_path: str) -> int:
    """Check that Fusion wrote a complete STEP containing every final body."""
    if not os.path.isfile(step_path):
        raise RuntimeError(
            f'STEP export reported success but no file exists: {step_path}'
        )

    file_size = os.path.getsize(step_path)
    if file_size < 1024:
        raise RuntimeError(
            f'STEP export is unexpectedly small ({file_size} bytes): {step_path}'
        )

    with open(step_path, 'r', encoding='utf-8', errors='ignore') as step_file:
        step_text = step_file.read()

    if 'ISO-10303-21;' not in step_text or 'END-ISO-10303-21;' not in step_text:
        raise RuntimeError(f'STEP file is incomplete or malformed: {step_path}')

    missing_names = [
        body_name for body_name in EXPECTED_BODY_NAMES
        if body_name not in step_text
    ]
    if missing_names:
        raise RuntimeError(
            'STEP export omitted expected bodies: ' + ', '.join(missing_names)
        )
    return file_size


def _export_complete_step(design: adsk.fusion.Design) -> str:
    """Export the single helix-baffle-cut watertight fluid body to STEP."""
    os.makedirs(EXPORT_DIRECTORY, exist_ok=True)
    step_path = os.path.join(EXPORT_DIRECTORY, STEP_FILENAME)
    temporary_step_path = os.path.join(
        EXPORT_DIRECTORY,
        'AnnularFluid_HelixBaffle_Watertight.exporting.step',
    )

    root_component = design.rootComponent
    actual_body_names = tuple(
        root_component.bRepBodies.item(index).name
        for index in range(root_component.bRepBodies.count)
    )
    missing_bodies = [
        body_name for body_name in EXPECTED_BODY_NAMES
        if body_name not in actual_body_names
    ]
    if missing_bodies:
        raise RuntimeError(
            'Cannot export because design bodies are missing: '
            + ', '.join(missing_bodies)
        )
    unexpected_bodies = [
        body_name for body_name in actual_body_names
        if body_name not in EXPECTED_BODY_NAMES
    ]
    if len(actual_body_names) != len(EXPECTED_BODY_NAMES) or unexpected_bodies:
        raise RuntimeError(
            'Cannot export because watertight-only mode requires exactly '
            f'one final body. Found: {", ".join(actual_body_names)}'
        )

    # Never validate a stale partial file left by a previous interrupted run.
    if os.path.exists(temporary_step_path):
        os.remove(temporary_step_path)

    # Give Fusion a chance to finish updating all transient graphics/model data
    # before the translation framework reads the root component.
    adsk.doEvents()

    export_manager = design.exportManager
    # Omitting the optional geometry argument is the documented way to export
    # the root component and all of its contents.
    options = export_manager.createSTEPExportOptions(temporary_step_path)
    if not options:
        raise RuntimeError('Fusion failed to create STEP export options.')
    if not export_manager.execute(options):
        raise RuntimeError(f'Fusion failed to export STEP: {temporary_step_path}')

    file_size = _validate_step_file(temporary_step_path)
    # Replace the old final file only after the new export passes validation.
    os.replace(temporary_step_path, step_path)
    _write_log(f'STEP file validated: {file_size} bytes, watertight body found.')
    return step_path


def run(context):
    try:
        _initialize_log()
        # Fusion can retain imported add-in modules between Stop/Run cycles.
        # Reload the modeling module so the file currently on disk is executed.
        importlib.reload(helix)
        _write_log(
            'Code revision: guide-rail sweep with native Combine fallback.'
        )

        # This will run the start function in each of your commands as defined in commands/__init__.py
        commands.start()
        _write_log('Add-in commands started.')

        # Generate and export only the helix-baffle-cut watertight fluid body.
        design = helix.build_double_pipe_with_helical_band()
        body_summary = ', '.join(
            _describe_body(design.rootComponent.bRepBodies.item(index))
            for index in range(design.rootComponent.bRepBodies.count)
        )
        _write_log(f'Geometry created: {body_summary}')

        step_path = _export_complete_step(design)
        _write_log(f'STEP export completed: {step_path}')
        adsk.core.Application.get().userInterface.messageBox(
            'Watertight annular-fluid geometry and STEP file created.\n\n'
            f'{step_path}'
        )

    except Exception:
        _write_log(f'CodexFusion run failed:\n{traceback.format_exc()}')
        futil.handle_error('run')


def stop(context):
    try:
        _write_log('CodexFusion stop started.')

        # Remove all of the event handlers your app has created
        futil.clear_handlers()

        # This will run the start function in each of your commands as defined in commands/__init__.py
        commands.stop()
        _write_log('CodexFusion stopped successfully.')

    except Exception:
        _write_log(f'CodexFusion stop failed:\n{traceback.format_exc()}')
        futil.handle_error('stop')
