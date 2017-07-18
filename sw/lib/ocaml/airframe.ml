module Include = struct

  type t = { href: string; xml: Xml.xml }

  let from_xml = function
    | Xml.Element ("include", [("href", href)], []) as xml -> { href; xml }
    | _ -> failwith "Airframe.Include.from_xml: unreachable"

end

module Firmware = struct

  type t = { name: string; xml: Xml.xml }

  let from_xml = function
    | Xml.Element ("firmware", [("name", name)], []) as xml -> { name; xml }
    | _ -> failwith "Airframe.Firmware.from_xml: unreachable"

end

module Autopilot = struct

  type t = { name: string; freq: float option; xml: Xml.xml }

  let from_xml = function
    | Xml.Element ("firmware", attribs, []) as xml ->
	{ name = List.assoc "name" attribs;
	  freq = Module.find_opt_map attribs "freq" float_of_string;
	  xml }
    | _ -> failwith "Airframe.Autopilot.from_xml: unreachable"

end

module Servo = struct

  type t = {
      name: string;
      number: int;
      driver: string option;
      min: float;
      neutral: float;
      max: float;
      xml: Xml.xml
    }

  let from_mxl driver = function
    | Xml.Element ("servo", attribs, []) as xml ->
	let get = fun attrib -> List.assoc attrib attribs in
	{ name = get "name";
	  number = int_of_string (get "no");
	  driver;
	  min = float_of_string (get "min");
	  neutral = float_of_string (get "neutral");
	  max = float_of_string (get "max");
	  xml }
    | _ -> failwith "Airframe.Servo.from_xml: unreachable"

  let fprint = fun ch s ->
    let travel_up = (s.max -. s.neutral) /. max_pprz
    and travel_down = (s.neutral -. s.min) /. max_pprz in
    Printf.fprintf ch "#define SERVO_%s %d\n" s.name s.number;
    Printf.fprintf ch "#define %s_NEUTRAL %g\n" s.name s.neutral;
    Printf.fprintf ch "#define %s_TRAVEL_UP %g\n" s.name travel_up;
    (* TODO: define_integer travel_up 16 *)
    Printf.fprintf ch "#define %s_TRAVEL_DOWN %g\n" s.name travel_down;
    (* TODO: define_integer travel_down 16 *)
    Printf.fprintf ch "#define %s_MAX %g\n" s.name s.max;
    Printf.fprintf ch "#define %s_MIN %g\n" s.name s.min;

end

type axis = { axis: string; failsafe_value: string }
type set
type ap_only_command
type command_law
type section
type makefile
type modul
type firmware
type autopilot
type heli_curve

type t = {
    includes: Include.t list;
    servos: Servo.t list;
    commands: axis list;
    rc_commands: set list;
    auto_rc_commands: set list;
    ap_only_commands: ap_only_command list;
    command_laws: command_law list;
    sections: section list;
    makefiles: makefile list;
    modules: modul list;
    firmwares: Firmware.t list;
    autopilots: Autopilot.t list;
    heli_curves: heli_curve list
  }
