(* duplicated code *)
let find_opt = fun k l -> try Some (List.assoc k l) with Not_found -> None
let find_opt_map = fun k l f ->
  try Some (f (List.assoc k l)) with Not_found -> None
let find_default = fun def k l -> try List.assoc k l with Not_found -> def
    (* end of duplicated code *)

module Autopilot = struct

  type t = { name: string; freq: float option; xml: Xml.xml }

  let from_xml = function
    | Xml.Element ("autopilot", attrs, []) as xml ->
        { name = List.assoc "name" attrs;
          freq = find_opt_map "freq" attrs float_of_string;
          xml }
    | _ -> failwith "Airframe.Autopilot.from_xml: unreachable"

end

module Include = struct

  type t = { href: string; xml: Xml.xml }

  let from_xml = function
    | Xml.Element ("include", [("href", href)], []) as xml -> { href; xml }
    | _ -> failwith "Airframe.Include.from_xml: unreachable"

end

module Configure = struct

  type t = { name: string;
             value: float;
             description: string option;
             xml: Xml.xml }

  let from_xml = function
    | Xml.Element ("configure", attrs, []) as xml ->
        { name = List.assoc "name" attrs;
          value = float_of_string (List.assoc "value" attrs);
          description = find_opt "description" attrs;
          xml }
    | _ -> failwith "Airframe.Configure.from_xml: unreachable"

end

module Define = struct

  type t = { name: string;
             value: float option;
             unit: string option;
             code_unit: string option;
             integer: int option;
             dtype: string option;
             description: string option;
             xml: Xml.xml }

  let from_xml = function
    | Xml.Element ("define", attrs, []) as xml ->
        { name = List.assoc "name" attrs;
          value = find_opt_map "value" attrs float_of_string;
          unit = find_opt "unit" attrs;
          code_unit = find_opt "code_unit" attrs;
          integer = find_opt_map "integer" attrs int_of_string;
          dtype = find_opt "type" attrs;
          description = find_opt "description" attrs;
          xml }
    | _ -> failwith "Airframe.Define.from_xml: unreachable"

end

module Comment = struct

  type t = { contents: string; xml: Xml.xml }

  let from_xml = function
    | Xml.Element ("comment", [], [Xml.PCData contents]) as xml ->
        { contents; xml }
    | _ -> failwith "Airframe.Comment.from_xml: unreachable"

end

module Module = struct

  type t = { name: string;
             mtype: string option;
             dir: string option;
             configures: Configure.t list;
             defines: Define.t list;
             comments: Comment.t list;
             xml: Xml.xml }

  let from_xml = function
    | Xml.Element ("module", attrs, children) as xml ->
        (* TODO *)
    | _ -> failwith "Airframe.Module.from_xml: unreachable"

end

module Target = struct

  type t = { name: string;
             board: string;
             processor: string option;
             modules: Module.t list;
             autopilot: Autopilot.t option;
             configures: Configure.t list;
             defines: Define.t list;
             comments: Comment.t list;
             xml: Xml.xml }

  let from_xml = function
    | Xml.Element ("target", attrs, []) as xml ->
        { name = List.assoc "name" attrs;
          board = List.assoc "board" attrs;
          processor = Module.find_opt attrs "processor";
          (* TODO *)
          xml }
    | _ -> failwith "Airframe.Autopilot.from_xml: unreachable"

end

module Firmware = struct

  type t = { name: string;
             targets: Target.t list;
             modules: Module.t list;
             autopilot: Autopilot.t option;
             configures: Configure.t list;
             defines: Define.t list;
             comments: Comment.t list;
             xml: Xml.xml }

  let from_xml = function
    | Xml.Element ("firmware", [("name", name)], children) as xml ->
        (* TODO *)
        { name; xml }
    | _ -> failwith "Airframe.Firmware.from_xml: unreachable"

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

  let from_xml driver = function
    | Xml.Element ("servo", attrs, []) as xml ->
        let get = fun attrib -> List.assoc attrib attrs in
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

module Axis = struct

  type t = { axis: string; failsafe_value: string; xml: Xml.xml }

  let from_xml = function
    | Xml.Element ("axis", attrs, []) as xml ->
        { name = List.assoc "name" attrs;
          failsafe_value = List.assoc "failsafe_value" attrs;
          xml }
    | _ -> failwith "Airframe.Axis.from_xml: unreachable"

end

module Set = struct

  type t = { value: float;
             command: string option;
             servo: string option;
             xml: Xml.xml }

  let from_xml = function
    | Xml.Element ("set", attrs, []) as xml ->
        { value = List.assoc "value" attrs;
          command = find_opt "command" attrs;
          servo = find_opt "servo" attrs;
          xml }
    | _ -> failwith "Airframe.Set.from_xml: unreachable"

end

module Call = struct

  type t = { func: string; xml: Xml.xml }

  let from_xml = function
    | Xml.Element ("call", [("fun", func)], []) as xml -> { func; xml }
    | _ -> failwith "Airframe.Call.from_xml: unreachable"

end

module Ratelimit = struct

  type t = { var: string;
             value: float;
             rate_min: float;
             rate_max: float;
             xml: Xml.xml }

  let from_xml = function
    | Xml.Element ("ratelimit", attrs, []) as xml ->
        { var = List.assoc "var" attrs;
          value = List.assoc "value" attrs;
          rate_min = List.assoc "rate_min" attrs;
          rate_max = List.assoc "rate_max" attrs;
          xml }
    | _ -> failwith "Airframe.Ratelimit.from_xml: unreachable"

end

module Copy = struct

  type t = { command: string; xml: Xml.xml }

  let from_xml = function
    | Xml.Element ("copy", [("command", command)], []) as xml ->
        { command; xml }
    | _ -> failwith "Airframe.Copy.from_xml: unreachable"

end

module Curve = struct

  type t = { throttle: int list;
             collective: int list;
             xml: Xml.xml }

  let from_xml = function
    | Xml.Element ("curve", attrs, []) as xml ->
        let str2intlist = fun s ->
          s |> Str.split (Str.regexp ",") |> List.map int_of_string in
        { throttle = List.assoc "throttle" attrs |> str2intlist;
          collective = List.assoc "collective" attrs |> str2intlist;
          xml }
    | _ -> failwith "Airframe.Curve.from_xml: unreachable"

end

module Let = struct

  type t = { var: string; value: float; xml: Xml.xml }

  let from_xml = function
    | Xml.Element ("let", attrs, []) as xml ->
        { var = List.assoc "var" attrs;
          value = List.assoc "value" attrs |> float_of_string;
          xml }
    | _ -> failwith "Airframe.Let.from_xml: unreachable"

end

module Makefile = struct

  type t = { target: string option;
             location: string option;
             contents: string;
             xml: Xml.xml }

  let from_xml = function
    | Xml.Element ("makefile", attrs, [Xml.PCData contents]) as xml ->
        { target = find_opt "target" attrs;
          location = find_opt "location" attrs;
          contents; xml }
    | _ -> failwith "Airframe.Makefile.from_xml: unreachable"

end

type ap_only_command
type command_law
type section
type modul
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
