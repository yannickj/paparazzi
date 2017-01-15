type incl

module Servo = struct

  type t = {
      name: string;
      number: int;
      driver: string option;
      min: float;
      neutral: float;
      max: float
    }

  let parse_mxl driver = function
    | Xml.Element ("servo", attribs, []) ->
	let get = fun attrib -> List.assoc attrib attribs in
	{ name = get "name";
	  number = int_of_string (get "no");
	  driver;
	  min = float_of_string (get "min");
	  neutral = float_of_string (get "neutral");
	  max = float_of_string (get "max") }
    | _ -> failwith "Airframe.Servo.parse_xml: unreachable"

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

type t = {
    includes: incl list;
    servos: servo list;
    commands: axis list;
    rc_commands: set list;
    auto_rc_commands: set list;
    ap_only_commands: ??? list;
    command_laws: ??? list;
    sections: ??? list;
    
  }
