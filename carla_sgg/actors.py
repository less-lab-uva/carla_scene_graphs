import carla

def get_attribute_value(attribute):
    type_dict = {
        carla.ActorAttributeType.Bool: attribute.as_bool,
        carla.ActorAttributeType.Int: attribute.as_int,
        carla.ActorAttributeType.Float: attribute.as_float,
        carla.ActorAttributeType.String: attribute.as_str,
        carla.ActorAttributeType.RGBColor: attribute.as_color
    }
    return type_dict[attribute.type]() if attribute.type in type_dict else None


def get_attribute(bp, attribute_name):
    return get_attribute_value(bp.get_attribute(attribute_name))\
          if bp.has_attribute(attribute_name) else None


def get_vehicles(world, base_type='car', special_type=None):
    return [car for car in world.get_blueprint_library().filter('vehicle.*')\
             if (base_type is None or\
                 get_attribute(car, "base_type") == base_type) and\
                (special_type is None or\
                get_attribute(car, "special_type") == special_type)]

def get_base_type(blueprint, strict_handling=False):
    if 'traffic.unknown' in blueprint:
        return None
    if 'traffic.speed_limit' in blueprint:
        return 'speed_limit'
    if 'static.prop' in blueprint:
        return None
    if 'walker' in blueprint:
        return 'person'
    if 'landmark' in blueprint:
        return blueprint
    if 'junction_boundary' in blueprint:
        return blueprint
    if blueprint in base_type_map:
        return base_type_map[blueprint]
    if strict_handling:
        raise NotImplementedError(f'Unknown class {blueprint}')
    else:
        print(f'Warning: found unknown class {blueprint}')
        return None

def get_special_type(blueprint):
    if blueprint in special_type_map:
        return special_type_map[blueprint]
    return ''

base_type_map = {
    # Vehicle
    'Vehicles': 'vehicle',  # in older versions of CARLA, static vehicles are not given a specific type, e.g. car, truck
    # cars
    'Car': 'car',  # handle the unlabelled map bg cars
    'vehicle.audi.a2': 'car',
    'vehicle.audi.etron': 'car',
    'vehicle.audi.tt': 'car',
    'vehicle.bmw.grandtourer': 'car',
    'vehicle.chevrolet.impala': 'car',
    'vehicle.citroen.c3': 'car',
    'vehicle.dodge.charger_2020': 'car',
    'vehicle.dodge.charger_police': 'car',
    'vehicle.dodge.charger_police_2020': 'car',
    'vehicle.ford.crown': 'car',
    'vehicle.ford.mustang': 'car',
    'vehicle.jeep.wrangler_rubicon': 'car',
    'vehicle.lincoln.mkz_2017': 'car',
    'vehicle.lincoln.mkz_2020': 'car',
    'vehicle.mercedes.coupe': 'car',
    'vehicle.mercedes.coupe_2020': 'car',
    'vehicle.micro.microlino': 'car',
    'vehicle.mini.cooper_s': 'car',
    'vehicle.mini.cooper_s_2021': 'car',
    'vehicle.nissan.micra': 'car',
    'vehicle.nissan.patrol': 'car',
    'vehicle.nissan.patrol_2021': 'car',
    'vehicle.seat.leon': 'car',
    'vehicle.tesla.model3': 'car',
    'vehicle.toyota.prius': 'car',
    # from older versions of CARLA:
    'vehicle.bmw.isetta': 'car',
    'vehicle.dodge_charger.police': 'car',
    'vehicle.mustang.mustang': 'car',
    'vehicle.lincoln.mkz2017': 'car',
    'vehicle.mercedes-benz.coupe': 'car',
    'vehicle.mini.cooperst': 'car',
    # trucks
    'Truck': 'truck',  # handle the unlabelled map bg trucks
    'vehicle.carlamotors.carlacola': 'truck',
    'vehicle.carlamotors.european_hgv': 'truck',
    'vehicle.carlamotors.firetruck': 'truck',
    'vehicle.tesla.cybertruck': 'truck',
    # vans
    'vehicle.ford.ambulance': 'van',  
    # ^ This "van" is from the line of Ambulances that use a modified F-series pickup as the base chassis. Unclear why CARLA calls it a "van", edit here to change
    'vehicle.mercedes.sprinter': 'van',
    'vehicle.volkswagen.t2': 'van',
    'vehicle.volkswagen.t2_2021': 'van',
    # buses
    'Bus': 'bus',  # handle the unlabelled background buses
    'vehicle.mitsubishi.fusorosa': 'bus',
    # motorcycles
    'Motorcycle': 'motorcycle',
    'vehicle.harley-davidson.low_rider': 'motorcycle',
    'vehicle.kawasaki.ninja': 'motorcycle',
    'vehicle.vespa.zx125': 'motorcycle',  # CARLA does not differentiate between moped and motorcycle -- edit here to change
    'vehicle.yamaha.yzf': 'motorcycle',
    # bicycles
    'Bicycle': 'bicycle',  # handle the unlabelled background bikes
    'vehicle.bh.crossbike': 'bicycle',
    'vehicle.diamondback.century': 'bicycle',
    'vehicle.gazelle.omafiets': 'bicycle',
    # traffic light
    'traffic.traffic_light': 'traffic_light',
    # stop sign
    'traffic.stop': 'stop_sign',
}
special_type_map = {
    # cars
    'vehicle.dodge.charger_police': 'emergency',
    'vehicle.dodge.charger_police_2020': 'emergency',
    'vehicle.ford.crown': 'taxi',
    'vehicle.micro.microlino': 'electric',
    'vehicle.tesla.model3': 'electric',
    'vehicle.toyota.prius': 'electric',
    # from older versions of CARLA
    'vehicle.dodge_charger.police': 'emergency',
    # trucks
    'vehicle.carlamotors.firetruck': 'emergency',
    'vehicle.tesla.cybertruck': 'electric',
    # vans
    'vehicle.ford.ambulance': 'emergency',
}