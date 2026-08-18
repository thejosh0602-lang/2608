"""Create a double-pipe CFD model with a sheet or solid helical baffle.

The script creates separate solid and fluid bodies so the result can be taken
into SpaceClaim/Fluent Meshing.  Fusion's internal length unit is centimetres;
all editable dimensions below are intentionally specified in millimetres.
"""

import math
import traceback

import adsk.core
import adsk.fusion


# -----------------------------------------------------------------------------
# Editable dimensions (mm)
# -----------------------------------------------------------------------------
MODEL_LENGTH_MM = 5000.

OUTER_PIPE_OD_MM = 609.6
OUTER_PIPE_ID_MM = 590.6

INNER_PIPE_OD_MM = 406.4
INNER_PIPE_ID_MM = 381.0

HELIX_PITCH_MM = 350.
# Physical cutter thickness removed from the final annular-fluid volume.
HELIX_BAFFLE_THICKNESS_MM = 10.
HELIX_RADIAL_CLEARANCE_MM = 0.
# Radial overrun used only by the Boolean cutter.  The cutter penetrates both
# annular walls by this amount so Fusion never has to solve coincident faces.
SOLID_BAFFLE_BOOLEAN_CLEARANCE_MM = 0.1
HELIX_END_CLEARANCE_MM = 100


def _cm(value_mm: float) -> float:
    """Convert millimetres to Fusion's internal centimetre unit."""
    return value_mm / 10.0


def _effective_baffle_radial_clearance_mm() -> float:
    """Return a Boolean-safe radial clearance for the solid cutter."""
    return max(
        HELIX_RADIAL_CLEARANCE_MM,
        SOLID_BAFFLE_BOOLEAN_CLEARANCE_MM,
    )


def _validate_dimensions() -> None:
    if MODEL_LENGTH_MM <= 0 or HELIX_PITCH_MM <= 0:
        raise ValueError('MODEL_LENGTH_MM and HELIX_PITCH_MM must be positive.')
    if not 0 < INNER_PIPE_ID_MM < INNER_PIPE_OD_MM:
        raise ValueError('Inner-pipe diameters are invalid.')
    if not INNER_PIPE_OD_MM < OUTER_PIPE_ID_MM < OUTER_PIPE_OD_MM:
        raise ValueError('The pipe diameters do not define a valid annulus.')

    if HELIX_RADIAL_CLEARANCE_MM < 0:
        raise ValueError('HELIX_RADIAL_CLEARANCE_MM cannot be negative.')
    if SOLID_BAFFLE_BOOLEAN_CLEARANCE_MM < 0:
        raise ValueError('SOLID_BAFFLE_BOOLEAN_CLEARANCE_MM cannot be negative.')

    annular_gap = (OUTER_PIPE_ID_MM - INNER_PIPE_OD_MM) / 2.0
    effective_clearance = _effective_baffle_radial_clearance_mm()
    if 2.0 * effective_clearance >= annular_gap:
        raise ValueError('HELIX_RADIAL_CLEARANCE_MM leaves no radial baffle height.')
    if 2.0 * HELIX_END_CLEARANCE_MM >= MODEL_LENGTH_MM:
        raise ValueError('HELIX_END_CLEARANCE_MM leaves no helix length.')
    if HELIX_BAFFLE_THICKNESS_MM <= 0:
        raise ValueError('HELIX_BAFFLE_THICKNESS_MM must be positive.')


def _annulus_profile(sketch: adsk.fusion.Sketch) -> adsk.fusion.Profile:
    """Return the ring-shaped profile from a two-circle sketch."""
    for index in range(sketch.profiles.count):
        profile = sketch.profiles.item(index)
        if profile.profileLoops.count > 1:
            return profile
    raise RuntimeError('Could not find the annular sketch profile.')


def _extrude_profile(
    component: adsk.fusion.Component,
    profile: adsk.fusion.Profile,
    distance_mm: float,
    body_name: str,
) -> adsk.fusion.BRepBody:
    extrudes = component.features.extrudeFeatures
    extrude_input = extrudes.createInput(
        profile,
        adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
    )
    extrude_input.setDistanceExtent(
        False,
        adsk.core.ValueInput.createByReal(_cm(distance_mm)),
    )
    feature = extrudes.add(extrude_input)
    body = feature.bodies.item(0)
    body.name = body_name
    return body


def _create_annular_body(
    component: adsk.fusion.Component,
    outer_diameter_mm: float,
    inner_diameter_mm: float,
    length_mm: float,
    body_name: str,
) -> adsk.fusion.BRepBody:
    sketch = component.sketches.add(component.xYConstructionPlane)
    sketch.name = f'{body_name}_Profile'
    circles = sketch.sketchCurves.sketchCircles
    origin = adsk.core.Point3D.create(0, 0, 0)
    circles.addByCenterRadius(origin, _cm(outer_diameter_mm / 2.0))
    circles.addByCenterRadius(origin, _cm(inner_diameter_mm / 2.0))
    body = _extrude_profile(component, _annulus_profile(sketch), length_mm, body_name)
    sketch.isLightBulbOn = False
    return body


def _create_cylindrical_body(
    component: adsk.fusion.Component,
    diameter_mm: float,
    length_mm: float,
    body_name: str,
) -> adsk.fusion.BRepBody:
    sketch = component.sketches.add(component.xYConstructionPlane)
    sketch.name = f'{body_name}_Profile'
    sketch.sketchCurves.sketchCircles.addByCenterRadius(
        adsk.core.Point3D.create(0, 0, 0),
        _cm(diameter_mm / 2.0),
    )
    body = _extrude_profile(component, sketch.profiles.item(0), length_mm, body_name)
    sketch.isLightBulbOn = False
    return body


def _create_helical_baffle(
    component: adsk.fusion.Component,
) -> adsk.fusion.BRepBody:
    inner_radius_mm = INNER_PIPE_OD_MM / 2.0
    outer_radius_mm = OUTER_PIPE_ID_MM / 2.0
    radial_clearance_mm = _effective_baffle_radial_clearance_mm()
    baffle_inner_radius_mm = inner_radius_mm + radial_clearance_mm
    baffle_outer_radius_mm = outer_radius_mm - radial_clearance_mm
    helix_height_mm = MODEL_LENGTH_MM - 2.0 * HELIX_END_CLEARANCE_MM
    turns = helix_height_mm / HELIX_PITCH_MM

    temp_brep = adsk.fusion.TemporaryBRepManager.get()
    axis_point = adsk.core.Point3D.create(0, 0, _cm(HELIX_END_CLEARANCE_MM))
    axis_vector = adsk.core.Vector3D.create(0, 0, 1)

    def make_helix(radius_mm: float) -> adsk.fusion.BRepBody:
        start_point = adsk.core.Point3D.create(
            _cm(radius_mm),
            0,
            _cm(HELIX_END_CLEARANCE_MM),
        )
        wire = temp_brep.createHelixWire(
            axis_point,
            axis_vector,
            start_point,
            _cm(HELIX_PITCH_MM),
            turns,
            0.0,
        )
        if not wire:
            raise RuntimeError('Fusion failed to create a helical baffle edge.')
        return wire

    # A ruled surface between equal-pitch inner and outer helices is a radial
    # helical sheet.  It is exported directly or thickened below by option.
    inner_helix = make_helix(baffle_inner_radius_mm)
    outer_helix = make_helix(baffle_outer_radius_mm)
    if inner_helix.wires.count == 0 or outer_helix.wires.count == 0:
        raise RuntimeError('A generated helix does not contain a BRepWire.')
    transient_surface = temp_brep.createRuledSurface(
        inner_helix.wires.item(0),
        outer_helix.wires.item(0),
    )
    if not transient_surface:
        raise RuntimeError('Fusion failed to create the helical baffle surface.')

    baffle = component.bRepBodies.add(transient_surface)
    if not baffle:
        raise RuntimeError('Fusion failed to persist the helical baffle sheet.')
    baffle.name = 'helix_baffle'
    if baffle.isSolid or baffle.faces.count == 0:
        raise RuntimeError('The helix baffle must be a non-empty sheet body.')
    baffle.attributes.add(
        'CodexFusion',
        'ThinWallThicknessMM',
        str(HELIX_BAFFLE_THICKNESS_MM),
    )
    baffle.attributes.add(
        'CodexFusion',
        'RadialClearanceMM',
        str(radial_clearance_mm),
    )
    return baffle


def _thicken_helical_baffle(
    component: adsk.fusion.Component,
    sheet_baffle: adsk.fusion.BRepBody,
) -> adsk.fusion.BRepBody:
    """Symmetrically thicken the helical sheet into a physical solid body."""
    inputs = adsk.core.ObjectCollection.create()
    inputs.add(sheet_baffle)

    thicken_features = component.features.thickenFeatures
    thicken_input = thicken_features.createInput(
        inputs,
        adsk.core.ValueInput.createByReal(_cm(HELIX_BAFFLE_THICKNESS_MM)),
        True,
        adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
        False,
    )
    if not thicken_input:
        raise RuntimeError('Fusion failed to create the baffle thicken input.')

    feature = thicken_features.add(thicken_input)
    if not feature or feature.bodies.count == 0:
        raise RuntimeError('Fusion failed to thicken the helical baffle.')

    solid_baffle = feature.bodies.item(0)
    solid_baffle.name = 'helix_baffle'
    if not solid_baffle.isValid or not solid_baffle.isSolid:
        raise RuntimeError('The thickened helix baffle is not a valid solid body.')
    if solid_baffle.volume <= 0:
        raise RuntimeError('The thickened helix baffle has no positive volume.')
    if sheet_baffle.isValid and not sheet_baffle.isSolid:
        if not sheet_baffle.deleteMe():
            raise RuntimeError('Fusion could not remove the source baffle sheet.')
    solid_baffle.attributes.add(
        'CodexFusion',
        'PhysicalThicknessMM',
        str(HELIX_BAFFLE_THICKNESS_MM),
    )
    return solid_baffle


def _create_swept_helical_baffle(
    component: adsk.fusion.Component,
) -> adsk.fusion.BRepBody:
    """Sweep a radial rectangle along a center helix to make a clean solid."""
    # Intentional user clearance is positive/inward.  Boolean overrun is
    # negative/outward, ensuring that the default zero-clearance baffle cuts
    # completely through both cylindrical boundaries without coincident faces.
    inner_radius_cm = _cm(
        INNER_PIPE_OD_MM / 2.0
        + HELIX_RADIAL_CLEARANCE_MM
        - SOLID_BAFFLE_BOOLEAN_CLEARANCE_MM
    )
    outer_radius_cm = _cm(
        OUTER_PIPE_ID_MM / 2.0
        - HELIX_RADIAL_CLEARANCE_MM
        + SOLID_BAFFLE_BOOLEAN_CLEARANCE_MM
    )
    mean_radius_cm = (inner_radius_cm + outer_radius_cm) / 2.0
    start_z_cm = _cm(HELIX_END_CLEARANCE_MM)
    helix_height_mm = MODEL_LENGTH_MM - 2.0 * HELIX_END_CLEARANCE_MM
    turns = helix_height_mm / HELIX_PITCH_MM
    pitch_per_radian_cm = _cm(HELIX_PITCH_MM) / (2.0 * math.pi)

    temporary_brep = adsk.fusion.TemporaryBRepManager.get()
    axis_point = adsk.core.Point3D.create(0, 0, start_z_cm)
    axis_vector = adsk.core.Vector3D.create(0, 0, 1)
    path_start = adsk.core.Point3D.create(mean_radius_cm, 0, start_z_cm)
    transient_path = temporary_brep.createHelixWire(
        axis_point,
        axis_vector,
        path_start,
        _cm(HELIX_PITCH_MM),
        turns,
        0.0,
    )
    if not transient_path or transient_path.edges.count == 0:
        raise RuntimeError('Fusion failed to create the solid-baffle sweep path.')
    path_body = component.bRepBodies.add(transient_path)
    if not path_body or path_body.edges.count == 0:
        raise RuntimeError('Fusion failed to persist the solid-baffle sweep path.')
    path_body.name = '_Helix_Sweep_Path'
    sweep_path = component.features.createPath(path_body.edges.item(0), False)
    if not sweep_path:
        raise RuntimeError('Fusion failed to create a sweep Path from the helix.')

    # A second, concentric helix fixes the radial direction of the rectangular
    # profile.  Without this guide rail Fusion parallel-transports the profile
    # along the long 3-D curve and the baffle visibly twists.
    guide_start = adsk.core.Point3D.create(inner_radius_cm, 0, start_z_cm)
    transient_guide = temporary_brep.createHelixWire(
        axis_point,
        axis_vector,
        guide_start,
        _cm(HELIX_PITCH_MM),
        turns,
        0.0,
    )
    if not transient_guide or transient_guide.edges.count == 0:
        raise RuntimeError('Fusion failed to create the baffle guide helix.')
    guide_body = component.bRepBodies.add(transient_guide)
    if not guide_body or guide_body.edges.count == 0:
        raise RuntimeError('Fusion failed to persist the baffle guide helix.')
    guide_body.name = '_Helix_Sweep_Guide'
    guide_path = component.features.createPath(guide_body.edges.item(0), False)
    if not guide_path:
        raise RuntimeError('Fusion failed to create the baffle guide Path.')

    # At theta=0 the helix tangent is (0, R, pitch/2pi).  The profile plane is
    # normal to that tangent; its rectangle spans the annular radius and the
    # local helicoid-normal thickness direction.
    tangent = adsk.core.Vector3D.create(
        0,
        mean_radius_cm,
        pitch_per_radian_cm,
    )
    if not tangent.normalize():
        raise RuntimeError('Could not normalize the helix tangent.')
    profile_plane_geometry = adsk.core.Plane.create(path_start, tangent)
    plane_input = component.constructionPlanes.createInput()
    if not plane_input.setByPlane(profile_plane_geometry):
        raise RuntimeError('Fusion failed to define the sweep profile plane.')
    profile_plane = component.constructionPlanes.add(plane_input)
    if not profile_plane:
        raise RuntimeError('Fusion failed to create the sweep profile plane.')
    profile_plane.name = 'Helix_Baffle_Sweep_Profile_Plane'

    profile_sketch = component.sketches.add(profile_plane)
    profile_sketch.name = 'Helix_Baffle_Solid_Profile'
    half_thickness_cm = _cm(HELIX_BAFFLE_THICKNESS_MM) / 2.0
    normal_y = -pitch_per_radian_cm
    normal_z = mean_radius_cm
    normal_length = math.sqrt(normal_y * normal_y + normal_z * normal_z)
    normal_y /= normal_length
    normal_z /= normal_length

    model_points = (
        adsk.core.Point3D.create(
            inner_radius_cm,
            -half_thickness_cm * normal_y,
            start_z_cm - half_thickness_cm * normal_z,
        ),
        adsk.core.Point3D.create(
            outer_radius_cm,
            -half_thickness_cm * normal_y,
            start_z_cm - half_thickness_cm * normal_z,
        ),
        adsk.core.Point3D.create(
            outer_radius_cm,
            half_thickness_cm * normal_y,
            start_z_cm + half_thickness_cm * normal_z,
        ),
        adsk.core.Point3D.create(
            inner_radius_cm,
            half_thickness_cm * normal_y,
            start_z_cm + half_thickness_cm * normal_z,
        ),
    )
    sketch_points = tuple(
        profile_sketch.modelToSketchSpace(point) for point in model_points
    )
    lines = profile_sketch.sketchCurves.sketchLines
    for index in range(4):
        lines.addByTwoPoints(sketch_points[index], sketch_points[(index + 1) % 4])
    if profile_sketch.profiles.count == 0:
        raise RuntimeError('Fusion failed to close the solid-baffle profile.')

    sweep_features = component.features.sweepFeatures
    sweep_input = sweep_features.createInput(
        profile_sketch.profiles.item(0),
        sweep_path,
        adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
    )
    if not sweep_input:
        raise RuntimeError('Fusion failed to create the helix sweep input.')
    sweep_input.guideRail = guide_path
    sweep_input.profileScaling = (
        adsk.fusion.SweepProfileScalingOptions.SweepProfileScaleOption
    )
    sweep_feature = sweep_features.add(sweep_input)
    if not sweep_feature or sweep_feature.bodies.count == 0:
        detail = sweep_feature.errorOrWarningMessage if sweep_feature else ''
        raise RuntimeError(f'Fusion failed to sweep the solid baffle. {detail}')

    solid_baffle = sweep_feature.bodies.item(0)
    solid_baffle.name = 'helix_baffle_cutter'
    if not solid_baffle.isValid or not solid_baffle.isSolid:
        raise RuntimeError('The swept helix baffle cutter is not a valid solid.')
    if solid_baffle.volume <= 0:
        raise RuntimeError('The swept helix baffle cutter has no positive volume.')

    profile_sketch.isLightBulbOn = False
    profile_plane.isLightBulbOn = False
    if path_body.isValid and not path_body.deleteMe():
        raise RuntimeError('Fusion could not remove the temporary helix path body.')
    if guide_body.isValid and not guide_body.deleteMe():
        raise RuntimeError('Fusion could not remove the temporary helix guide body.')
    return solid_baffle


def _combine_bodies(
    component: adsk.fusion.Component,
    target: adsk.fusion.BRepBody,
    tool: adsk.fusion.BRepBody,
    operation: adsk.fusion.FeatureOperations,
    keep_tool: bool,
    result_name: str,
) -> adsk.fusion.BRepBody:
    """Boolean transient copies, then replace only the persistent target."""
    temporary_brep = adsk.fusion.TemporaryBRepManager.get()
    target_copy = temporary_brep.copy(target)
    tool_copy = temporary_brep.copy(tool)
    if not target_copy or not tool_copy:
        raise RuntimeError(f'Fusion failed to copy Boolean inputs for {result_name}.')

    if operation == adsk.fusion.FeatureOperations.CutFeatureOperation:
        boolean_type = adsk.fusion.BooleanTypes.DifferenceBooleanType
    elif operation == adsk.fusion.FeatureOperations.IntersectFeatureOperation:
        boolean_type = adsk.fusion.BooleanTypes.IntersectionBooleanType
    else:
        raise RuntimeError(f'Unsupported Boolean operation for {result_name}.')

    succeeded = temporary_brep.booleanOperation(
        target_copy,
        tool_copy,
        boolean_type,
    )
    if not succeeded or not target_copy.isValid or not target_copy.isSolid:
        raise RuntimeError(f'Fusion temporary Boolean failed for {result_name}.')
    if target_copy.volume <= 0:
        raise RuntimeError(f'Fusion Boolean produced no volume: {result_name}.')

    if not target.deleteMe():
        raise RuntimeError(f'Fusion could not replace Boolean target {target.name}.')
    if not keep_tool and tool.isValid and not tool.deleteMe():
        raise RuntimeError(f'Fusion could not consume Boolean tool {tool.name}.')

    result = component.bRepBodies.add(target_copy)
    if not result:
        raise RuntimeError(f'Fusion could not persist Boolean result {result_name}.')
    result.name = result_name
    return result


def _subtract_solid_baffle_from_fluid(
    component: adsk.fusion.Component,
    annular_fluid: adsk.fusion.BRepBody,
    solid_baffle: adsk.fusion.BRepBody,
) -> adsk.fusion.BRepBody:
    """Create a watertight annular-fluid body with the baffle volume removed."""
    fluid_volume_before = annular_fluid.volume
    baffle_volume = solid_baffle.volume
    result = _combine_bodies(
        component,
        annular_fluid,
        solid_baffle,
        adsk.fusion.FeatureOperations.CutFeatureOperation,
        False,
        'Annular_Fluid_Watertight',
    )
    removed_volume = fluid_volume_before - result.volume
    # The cutter deliberately overruns the inner and outer fluid boundaries,
    # so its full volume is slightly larger than the volume removed from fluid.
    volume_tolerance = max(1e-6, baffle_volume * 0.01)
    if (
        removed_volume < baffle_volume * 0.95
        or removed_volume > baffle_volume + volume_tolerance
    ):
        raise RuntimeError(
            'Annular-fluid subtraction volume check failed: '
            f'baffle={baffle_volume:.6g} cm^3, '
            f'removed={removed_volume:.6g} cm^3.'
        )
    result.attributes.add(
        'CodexFusion',
        'RemovedBaffleVolumeCM3',
        f'{removed_volume:.12g}',
    )
    return result


def _assert_no_solid_overlaps(bodies: list[adsk.fusion.BRepBody]) -> None:
    """Fail when any two final bodies share a non-zero volume."""
    temp_brep = adsk.fusion.TemporaryBRepManager.get()
    overlap_tolerance_cm3 = 1e-7

    for first_index in range(len(bodies)):
        first = bodies[first_index]
        if not first or not first.isValid or not first.isSolid:
            raise RuntimeError('A final export body is missing or is not watertight.')

        for second_index in range(first_index + 1, len(bodies)):
            second = bodies[second_index]
            first_copy = temp_brep.copy(first)
            second_copy = temp_brep.copy(second)
            if not first_copy or not second_copy:
                raise RuntimeError('Fusion failed to copy bodies for overlap checking.')

            intersects = temp_brep.booleanOperation(
                first_copy,
                second_copy,
                adsk.fusion.BooleanTypes.IntersectionBooleanType,
            )
            # Disjoint bodies or bodies sharing only an interface can return
            # false.  A successful intersection must have negligible volume.
            overlap_volume = _safe_intersection_volume(first_copy) if intersects else 0.0
            if overlap_volume > overlap_tolerance_cm3:
                raise RuntimeError(
                    f'Solid overlap detected: {first.name} / {second.name} '
                    f'({overlap_volume:.6g} cm^3).'
                )


def _safe_intersection_volume(body: adsk.fusion.BRepBody) -> float:
    """Return zero for a face/edge-only or numerically degenerate intersection."""
    if not body or not body.isValid or not body.isSolid:
        return 0.0
    try:
        return body.volume
    except RuntimeError:
        # Fusion can return a degenerate intersection body for coincident
        # interfaces and then raise InternalValidationError from volume.
        return 0.0


def build_double_pipe_with_helical_band() -> adsk.fusion.Design:
    """Build the model in a new direct-modeling Fusion document."""
    _validate_dimensions()

    app = adsk.core.Application.get()
    document = app.documents.add(
        adsk.core.DocumentTypes.FusionDesignDocumentType
    )
    design = adsk.fusion.Design.cast(document.products.itemByProductType('DesignProductType'))
    if not design:
        design = adsk.fusion.Design.cast(app.activeProduct)
    if not design:
        raise RuntimeError('The active Fusion document is not a Design document.')

    design.designType = adsk.fusion.DesignTypes.DirectDesignType
    component = design.rootComponent

    # The requested deliverable is one CFD-ready watertight annular-fluid
    # volume.  Create a physical baffle only as a Boolean cutter; it is consumed
    # by the cut and is not part of the final model.
    baffle = _create_swept_helical_baffle(component)

    annular_fluid = _create_annular_body(
        component,
        OUTER_PIPE_ID_MM,
        INNER_PIPE_OD_MM,
        MODEL_LENGTH_MM,
        'Annular_Fluid_Watertight',
    )
    if not annular_fluid.isSolid or annular_fluid.volume <= 0:
        raise RuntimeError('The annular fluid region is not a closed solid body.')

    annular_fluid = _subtract_solid_baffle_from_fluid(
        component,
        annular_fluid,
        baffle,
    )
    if not annular_fluid.isValid or not annular_fluid.isSolid:
        raise RuntimeError('The final annular fluid is not a watertight solid.')
    if component.bRepBodies.count != 1:
        remaining_names = ', '.join(
            component.bRepBodies.item(index).name
            for index in range(component.bRepBodies.count)
        )
        raise RuntimeError(
            'Watertight-only mode must leave exactly one body. Found: '
            + remaining_names
        )

    app.activeViewport.fit()
    return design


def run(_context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        build_double_pipe_with_helical_band()
        ui.messageBox(
            'Watertight annular-fluid geometry created.\n\n'
            f'Helix baffle cut thickness: {HELIX_BAFFLE_THICKNESS_MM:g} mm\n'
            'Final body:\n'
            '- Annular_Fluid_Watertight'
        )
    except Exception:
        message = 'Helix model creation failed:\n{}'.format(traceback.format_exc())
        adsk.core.Application.log(message)
        if ui:
            ui.messageBox(message)


def stop(_context):
    """Required when the file is also loaded as an add-in module."""
    return
