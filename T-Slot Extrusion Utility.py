# T-Slot Extrusion Utility (fixed)
# - Two-sketch method (clean profile selection)
# - Symmetric solid extrude (keeps XY mid-length)
# - Slot cuts with symmetric distance > half-length (version-safe "through all")
# - Center bore and end-tap pilots sketched on END FACES (always intersects)
# - No participantBodies (avoids vector type mismatch across builds)
# Author: ChatGPT (GPT-5 Thinking)

import adsk.core, adsk.fusion, adsk.cam, traceback, math

_app = None
_ui  = None

# ---- Profile Library ---------------------------------------------------------
PROFILES = {
    '80/20 1010 (1" x 1")': {
        'unit': 'in', 'width': 1.0, 'height': 1.0,
        'slot_center_from_face': 0.25,
        'slot_depth': 0.28, 'slot_neck': 0.26, 'slot_open': 0.20,
        'center_bore_d': 0.201, 'end_tap_d': 0.159,
    },
    '80/20 1020 (1" x 2")': {
        'unit': 'in', 'width': 2.0, 'height': 1.0,
        'slot_center_from_face': 0.25,
        'slot_depth': 0.28, 'slot_neck': 0.26, 'slot_open': 0.20,
        'center_bore_d': 0.201, 'end_tap_d': 0.159,
    },
    '80/20 1515 (1.5" x 1.5")': {
        'unit': 'in', 'width': 1.5, 'height': 1.5,
        'slot_center_from_face': 0.375,
        'slot_depth': 0.40, 'slot_neck': 0.32, 'slot_open': 0.26,
        'center_bore_d': 0.257, 'end_tap_d': 0.257,
    },
    '80/20 1530 (1.5" x 3.0")': {
        'unit': 'in', 'width': 3.0, 'height': 1.5,
        'slot_center_from_face': 0.375,
        'slot_depth': 0.40, 'slot_neck': 0.32, 'slot_open': 0.26,
        'center_bore_d': 0.257, 'end_tap_d': 0.257,
    },
    '80/20 25-2525 (25 x 25 mm)': {
        'unit': 'mm', 'width': 25.0, 'height': 25.0,
        'slot_center_from_face': 6.5,
        'slot_depth': 7.5, 'slot_neck': 6.0, 'slot_open': 5.0,
        'center_bore_d': 5.2, 'end_tap_d': 4.2,
    },
    '80/20 25-2550 (25 x 50 mm)': {
        'unit': 'mm', 'width': 50.0, 'height': 25.0,
        'slot_center_from_face': 6.5,
        'slot_depth': 7.5, 'slot_neck': 6.0, 'slot_open': 5.0,
        'center_bore_d': 5.2, 'end_tap_d': 4.2,
    },
    'Misumi 3030 (30 x 30 mm)': {
        'unit': 'mm', 'width': 30.0, 'height': 30.0,
        'slot_center_from_face': 7.5,
        'slot_depth': 8.5, 'slot_neck': 6.8, 'slot_open': 6.0,
        'center_bore_d': 5.5, 'end_tap_d': 4.5,
    },
    'Misumi 4545 (45 x 45 mm)': {
        'unit': 'mm', 'width': 45.0, 'height': 45.0,
        'slot_center_from_face': 10.5,
        'slot_depth': 11.0, 'slot_neck': 8.2, 'slot_open': 7.0,
        'center_bore_d': 6.8, 'end_tap_d': 6.8,
    },
    'Bosch 30x30 (30 x 30 mm)': {
        'unit': 'mm', 'width': 30.0, 'height': 30.0,
        'slot_center_from_face': 7.5,
        'slot_depth': 8.5, 'slot_neck': 6.8, 'slot_open': 6.0,
        'center_bore_d': 5.5, 'end_tap_d': 4.5,
    },
    'Bosch 45x45 (45 x 45 mm)': {
        'unit': 'mm', 'width': 45.0, 'height': 45.0,
        'slot_center_from_face': 10.5,
        'slot_depth': 11.0, 'slot_neck': 8.2, 'slot_open': 7.0,
        'center_bore_d': 6.8, 'end_tap_d': 6.8,
    },
}

# ---- Dialog Defaults ---------------------------------------------------------
DEFAULT_PROFILE = '80/20 1010 (1" x 1")'
DEFAULT_LENGTH_MM = 500.0
DEFAULT_CENTER_BORE = False
DEFAULT_END_TAPS = False
DEFAULT_CONSTRUCTION = True

_handlers = []

# ---- Helpers ----------------------------------------------------------------
def toDocUnits(val, unitSymbol, des):
    um = des.unitsManager
    expr = f'{val} in' if unitSymbol == 'in' else f'{val} mm'
    return um.evaluateExpression(expr, um.defaultLengthUnits)

def draw_outer_rect(sketch, prof):
    ln = sketch.sketchCurves.sketchLines
    des = adsk.fusion.Design.cast(_app.activeProduct)

    w = toDocUnits(prof['width'],  prof['unit'], des)
    h = toDocUnits(prof['height'], prof['unit'], des)
    x = w * 0.5; y = h * 0.5

    p1 = adsk.core.Point3D.create(-x, -y, 0)
    p2 = adsk.core.Point3D.create( x, -y, 0)
    p3 = adsk.core.Point3D.create( x,  y, 0)
    p4 = adsk.core.Point3D.create(-x,  y, 0)
    ln.addByTwoPoints(p1, p2); ln.addByTwoPoints(p2, p3)
    ln.addByTwoPoints(p3, p4); ln.addByTwoPoints(p4, p1)

    profs = sketch.profiles
    outer_prof, max_area = None, -1.0
    for i in range(profs.count):
        p = profs.item(i)
        a = abs(p.areaProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy).area)
        if a > max_area and a < 1e8:  # ignore unbounded outside
            max_area = a
            outer_prof = p
    return outer_prof, w, h

def draw_slots(sketch, prof):
    ln = sketch.sketchCurves.sketchLines
    des = adsk.fusion.Design.cast(_app.activeProduct)

    w = toDocUnits(prof['width'],  prof['unit'], des)
    h = toDocUnits(prof['height'], prof['unit'], des)
    slot_depth = toDocUnits(prof['slot_depth'], prof['unit'], des)
    neck_w    = toDocUnits(prof['slot_neck'], prof['unit'], des)
    open_w    = toDocUnits(prof['slot_open'], prof['unit'], des)

    x = w * 0.5; y = h * 0.5
    d_open = min(slot_depth * 0.35, slot_depth - 1e-6)

    def rect(x1,y1,x2,y2):
        ln.addByTwoPoints(adsk.core.Point3D.create(x1,y1,0), adsk.core.Point3D.create(x2,y1,0))
        ln.addByTwoPoints(adsk.core.Point3D.create(x2,y1,0), adsk.core.Point3D.create(x2,y2,0))
        ln.addByTwoPoints(adsk.core.Point3D.create(x2,y2,0), adsk.core.Point3D.create(x1,y2,0))
        ln.addByTwoPoints(adsk.core.Point3D.create(x1,y2,0), adsk.core.Point3D.create(x1,y1,0))

    # Right (face x=+x, inward -X)
    rect( x, -open_w/2,  x - d_open,  open_w/2)
    rect( x - d_open, -neck_w/2,  x - slot_depth,  neck_w/2)
    # Left  (face x=-x, inward +X)
    rect(-x, -open_w/2, -x + d_open,  open_w/2)
    rect(-x + d_open, -neck_w/2, -x + slot_depth,  neck_w/2)
    # Top   (face y=+y, inward -Y)
    rect(-open_w/2,  y,  open_w/2,  y - d_open)
    rect(-neck_w/2,  y - d_open,  neck_w/2,  y - slot_depth)
    # Bottom(face y=-y, inward +Y)
    rect(-open_w/2, -y,  open_w/2, -y + d_open)
    rect(-neck_w/2, -y + d_open,  neck_w/2, -y + slot_depth)

    coll = adsk.core.ObjectCollection.create()
    profs = sketch.profiles
    area_cap = 3.0 * float(neck_w * slot_depth)  # keep to likely slot islands
    for i in range(profs.count):
        p = profs.item(i)
        a = abs(p.areaProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy).area)
        if 0.0 < a < area_cap:
            coll.add(p)
    return coll

def create_construction_for_slots(comp, prof):
    des = adsk.fusion.Design.cast(_app.activeProduct)
    unit = prof['unit']
    w = toDocUnits(prof['width'], unit, des)
    h = toDocUnits(prof['height'], unit, des)
    off = toDocUnits(prof['slot_center_from_face'], unit, des)

    planes = comp.constructionPlanes
    axes  = comp.constructionAxes

    # X slots (left/right)
    for sx in (-w/2 + off, w/2 - off):
        axisInput = axes.createInput()
        sp = comp.sketches.add(comp.xYConstructionPlane)
        pt = sp.sketchPoints.add(adsk.core.Point3D.create(sx, 0, 0))
        pt2 = sp.sketchPoints.add(adsk.core.Point3D.create(sx, 0, 1))
        axisInput.setByTwoPoints(pt, pt2)
        axes.add(axisInput)

    # Y slots (top/bottom)
    for sy in (-h/2 + off, h/2 - off):
        axisInput = axes.createInput()
        sp = comp.sketches.add(comp.xYConstructionPlane)
        pt = sp.sketchPoints.add(adsk.core.Point3D.create(0, sy, 0))
        pt2 = sp.sketchPoints.add(adsk.core.Point3D.create(0, sy, 1))
        axisInput.setByTwoPoints(pt, pt2)
        axes.add(axisInput)

    yz = comp.yZConstructionPlane
    xz = comp.xZConstructionPlane
    for sx in (-w/2 + off, w/2 - off):
        ip = planes.createInput(); ip.setByOffset(yz, adsk.core.ValueInput.createByReal(sx)); planes.add(ip)
    for sy in (-h/2 + off, h/2 - off):
        ip = planes.createInput(); ip.setByOffset(xz, adsk.core.ValueInput.createByReal(sy)); planes.add(ip)

def add_center_bore_and_end_taps(comp, prof, length, makeCenterBore, makeEndTaps):
    des = adsk.fusion.Design.cast(_app.activeProduct)
    unit = prof['unit']
    center_d = toDocUnits(prof['center_bore_d'], unit, des)
    endtap_d = toDocUnits(prof['end_tap_d'], unit, des)

    bodies = comp.bRepBodies
    if bodies.count == 0:
        return
    body = bodies.item(0)

    # --- Center bore: sketch on an end face, one-sided distance > length ---
    if makeCenterBore:
        zAxis = adsk.core.Vector3D.create(0, 0, 1)
        posFace, negFace = None, None
        for f in body.faces:
            if isinstance(f.geometry, adsk.core.Plane):
                n = f.geometry.normal
                if n.isParallelTo(zAxis):
                    if n.dotProduct(zAxis) > 0.0 and posFace is None: posFace = f
                    elif n.dotProduct(zAxis) < 0.0 and negFace is None: negFace = f
        startFace = posFace or negFace
        if not startFace:
            raise RuntimeError('No planar end face found for center bore.')

        sk = comp.sketches.add(startFace)
        circles = sk.sketchCurves.sketchCircles
        circles.addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), center_d/2.0)

        inner_prof, min_area = None, None
        profs = sk.profiles
        for i in range(profs.count):
            p = profs.item(i)
            try:
                a = abs(p.areaProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy).area)
            except:
                continue
            if a <= 0: continue
            if (min_area is None) or (a < min_area):
                min_area = a; inner_prof = p
        if inner_prof is None:
            raise RuntimeError('Center-bore profile not found.')

        w_doc = toDocUnits(prof['width'], unit, des)
        h_doc = toDocUnits(prof['height'], unit, des)
        safe_depth = adsk.core.ValueInput.createByReal(float(length) + 2.0 * max(w_doc, h_doc))

        ext = comp.features.extrudeFeatures
        extInput = ext.createInput(inner_prof, adsk.fusion.FeatureOperations.CutFeatureOperation)
        extInput.setDistanceExtent(False, safe_depth)  # one-sided deep cut
        ext.add(extInput)

    # --- End-tap pilots: sketch on both end faces, short distance cut ---
    if makeEndTaps:
        zAxis = adsk.core.Vector3D.create(0, 0, 1)
        endFaces = []
        for f in body.faces:
            if isinstance(f.geometry, adsk.core.Plane):
                n = f.geometry.normal
                if n.isParallelTo(zAxis):
                    endFaces.append(f)
        if len(endFaces) >= 1:
            pilotDepth = des.unitsManager.evaluateExpression('20 mm', des.unitsManager.defaultLengthUnits)
            for f in endFaces:
                sk = comp.sketches.add(f)
                circles = sk.sketchCurves.sketchCircles
                circles.addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), endtap_d/2.0)

                inner_prof, min_area = None, None
                profs = sk.profiles
                for i in range(profs.count):
                    p = profs.item(i)
                    try:
                        a = abs(p.areaProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy).area)
                    except:
                        continue
                    if a <= 0: continue
                    if (min_area is None) or (a < min_area):
                        min_area = a; inner_prof = p
                if inner_prof is None:
                    continue

                ext = comp.features.extrudeFeatures
                extInput = ext.createInput(inner_prof, adsk.fusion.FeatureOperations.CutFeatureOperation)
                extInput.setDistanceExtent(False, adsk.core.ValueInput.createByReal(pilotDepth))
                ext.add(extInput)

def apply_appearance_and_name(comp, profName, length_val):
    comp.name = f'{profName} - L={round(length_val,2)}'
    if comp.bRepBodies.count > 0:
        comp.bRepBodies.item(0).name = 'Extrusion'
    try:
        des = adsk.fusion.Design.cast(_app.activeProduct)
        appLib = _app.materialLibraries.itemByName('Fusion 360 Appearance Library')
        if appLib:
            app = appLib.appearances.itemByName('Aluminum - Anodized (Clear)') or appLib.appearances.itemByName('Aluminum - Satin')
            if app and comp.appearance is None:
                comp.appearance = app
    except:
        pass

# ---- Command UI / Event Handlers --------------------------------------------
class CommandExecuteHandler(adsk.core.CommandEventHandler):
    def __init__(self): super().__init__(); _handlers.append(self)
    def notify(self, args):
        try:
            des = adsk.fusion.Design.cast(_app.activeProduct)
            if not des:
                _ui.messageBox('Please switch to the Design workspace.'); return

            cmd = args.firingEvent.sender
            inputs = cmd.commandInputs

            dd = adsk.core.DropDownCommandInput.cast(inputs.itemById('profile'))
            profileName = dd.selectedItem.name if dd and dd.selectedItem else DEFAULT_PROFILE
            profile = PROFILES[profileName]

            lenInput = adsk.core.ValueCommandInput.cast(inputs.itemById('length'))
            length_val = lenInput.value

            cbCenter = adsk.core.BoolValueCommandInput.cast(inputs.itemById('centerBore'))
            cbEnd = adsk.core.BoolValueCommandInput.cast(inputs.itemById('endTaps'))
            cbConst = adsk.core.BoolValueCommandInput.cast(inputs.itemById('construct'))

            root = des.rootComponent
            occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            comp = adsk.fusion.Component.cast(occ.component)

            # Sketch 1: outer bar
            sk_outer = comp.sketches.add(comp.xYConstructionPlane)
            outerProf, w, h = draw_outer_rect(sk_outer, profile)

            # Extrude solid (symmetric about sketch plane)
            ext = comp.features.extrudeFeatures
            extInput = ext.createInput(outerProf, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
            half = adsk.core.ValueInput.createByReal(length_val/2.0)
            extInput.setSymmetricExtent(half, True)
            ext.add(extInput)

            # Sketch 2: slots (on same plane), then symmetric distance cut > half-length
            sk_slots = comp.sketches.add(comp.xYConstructionPlane)
            slotProfiles = draw_slots(sk_slots, profile)
            if slotProfiles and slotProfiles.count > 0:
                depth = adsk.core.ValueInput.createByReal((length_val/2.0) + max(w, h))
                cutInput = ext.createInput(slotProfiles, adsk.fusion.FeatureOperations.CutFeatureOperation)
                cutInput.setSymmetricExtent(depth, True)
                ext.add(cutInput)

            if cbConst.value:
                create_construction_for_slots(comp, profile)

            add_center_bore_and_end_taps(comp, profile, length_val, cbCenter.value, cbEnd.value)
            apply_appearance_and_name(comp, profileName, length_val)

            _ui.messageBox('T-slot extrusion created successfully.')
        except:
            if _ui: _ui.messageBox('Execute Failed:\n{}'.format(traceback.format_exc()))

class CommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    def __init__(self): super().__init__(); _handlers.append(self)
    def notify(self, args):
        try:
            cmd = adsk.core.Command.cast(args.command)
            cmd.isRepeatable = True
            cmd.execute.add(CommandExecuteHandler())
            cmd.destroy.add(CommandDestroyHandler('TSlotExtrusionUtility'))

            inputs = cmd.commandInputs
            dd = inputs.addDropDownCommandInput('profile', 'Profile', adsk.core.DropDownStyles.TextListDropDownStyle)
            for name in PROFILES.keys():
                dd.listItems.add(name, name == DEFAULT_PROFILE, '')
            dd.tooltip = 'Choose a standard extrusion profile.'

            des = adsk.fusion.Design.cast(_app.activeProduct)
            um = des.unitsManager if des else None
            defaultLen = DEFAULT_LENGTH_MM
            if um: defaultLen = um.evaluateExpression(f'{DEFAULT_LENGTH_MM} mm', um.defaultLengthUnits)
            inputs.addValueInput('length', 'Length', des.unitsManager.defaultLengthUnits if des else 'mm',
                                 adsk.core.ValueInput.createByReal(defaultLen))

            inputs.addBoolValueInput('centerBore', 'Add center bore (through)', True, '', DEFAULT_CENTER_BORE)
            inputs.addBoolValueInput('endTaps', 'Add end tap pilot holes (both ends)', True, '', DEFAULT_END_TAPS)
            inputs.addBoolValueInput('construct', 'Add construction axes/planes at slot centerlines', True, '', DEFAULT_CONSTRUCTION)
        except:
            if _ui: _ui.messageBox('CommandCreated Failed:\n{}'.format(traceback.format_exc()))

class CommandDestroyHandler(adsk.core.CommandEventHandler):
    def __init__(self, cmdDefId): super().__init__(); _handlers.append(self); self._cmdDefId = cmdDefId
    def notify(self, args: adsk.core.CommandEventArgs):
        try:
            if _ui:
                cmdDef = _ui.commandDefinitions.itemById(self._cmdDefId)
                if cmdDef: cmdDef.deleteMe()
        except: pass
        adsk.terminate()

# ---- Script entry/exit -------------------------------------------------------
def run(context):
    global _app, _ui
    try:
        _app = adsk.core.Application.get()
        _ui = _app.userInterface

        cmdDef = _ui.commandDefinitions.itemById('TSlotExtrusionUtility')
        if not cmdDef:
            cmdDef = _ui.commandDefinitions.addButtonDefinition(
                'TSlotExtrusionUtility',
                'T-Slot Extrusion Utility',
                'Create a standard aluminum T-slot extrusion with adjustable length and options.'
            )

        cmdDef.commandCreated.add(CommandCreatedHandler())
        cmdDef.execute()
        adsk.autoTerminate(False)
    except:
        if _ui: _ui.messageBox('Run Failed:\n{}'.format(traceback.format_exc()))

def stop(context):
    pass
