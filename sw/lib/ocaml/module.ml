(*
 * Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *                    Cyril Allignol <cyril.allignol@enac.fr>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *)

(**
 * 'Module' module for parsing XML config files
 *)

module OT = Ocaml_tools

let find_name = fun attribs ->
  try
    let name = List.assoc "name" attribs in
    if Filename.check_suffix name ".xml" then Filename.chop_extension name
    else name
  with
  | Not_found ->
      let msg = Printf.sprintf "Error: Attribute 'name' expected in %a"
          ExtXml.sprint_fields attribs in
      raise (ExtXml.Error msg)

type file = { filename: string; directory: string option }
type file_arch = file

let parse_file = function
  | Xml.Element ("file", attribs, []) | Xml.Element ("file_arch", attribs, []) ->
      { filename = find_name attribs; directory = OT.assoc_opt "dir" attribs }
  | _ -> failwith "Module.parse_file: unreachable"

type configure = {
    cname: string;
    cvalue: string option;
    case: string option;
    default: string option;
    cdescription: string option
  }

let parse_configure = fun attribs ->
  let get = fun x -> OT.assoc_opt x attribs in
  { cname = find_name attribs; cvalue = get "value"; case = get "case";
    default = get "default"; cdescription = get "description" }

type define = {
    dname: string;
    dvalue: string option;
    integer: int option;
    dunit: string option;
    dtype: string option;
    ddescription: string option;
    cond: string option
  }

let parse_define = fun attribs ->
  let get = fun x -> OT.assoc_opt x attribs in
  { dname = find_name attribs; dvalue = get "value";
    integer = begin match get "integer" with
    | None -> None | Some i -> Some (int_of_string i) end;
    dunit = get "unit"; dtype = get "type";
    ddescription = get "description"; cond = get "cond" }

type incl = { element: string; condition: string option }

type flag = { flag: string; value: string }

type raw = string

type makefile = {
    targets: string option;
    firmware: string option;
    condition: string option;
    configurations: configure list;
    definitions: define list;
    inclusions: incl list;
    flags: flag list;
    files: file list;
    files_arch: file list;
    raws: raw list
  }

let empty_makefile =
  { targets = None; firmware = None; condition = None; configurations = []; definitions = [];
    inclusions = []; flags = []; files = []; files_arch = []; raws = [] }

let rec parse_makefile mkf = function
  | Xml.Element ("makefile", attribs, children) ->
      let targets = OT.assoc_opt "target" attribs
      and firmware = OT.assoc_opt "firmware" attribs
      and condition = OT.assoc_opt "cond" attribs in
      List.fold_left parse_makefile { mkf with targets; firmware; condition } children
  | Xml.Element ("configure", attribs, []) ->
      { mkf with configurations = parse_configure attribs :: mkf.configurations }
  | Xml.Element ("define", attribs, []) ->
      { mkf with definitions = parse_define attribs :: mkf.definitions }
  | Xml.Element ("include", attribs, []) ->
      { mkf with inclusions =
        { element = find_name attribs; condition = OT.assoc_opt "cond" attribs }
        :: mkf.inclusions }
  | Xml.Element ("flag", [("name", flag); ("value", value)], [])
  | Xml.Element ("flag", [("value", value); ("name", flag)], []) ->
      { mkf with flags = { flag; value } :: mkf.flags }
  | Xml.Element ("file", _, []) as xml ->
      { mkf with files = parse_file xml :: mkf.files }
  | Xml.Element ("file_arch", _, []) as xml ->
      { mkf with files_arch = parse_file xml :: mkf.files_arch }
  | Xml.Element ("raw", [], [Xml.PCData raw]) -> {mkf with raws = raw :: mkf.raws}
  | _ -> failwith "Module.parse_makefile: unreachable"

type autorun = True | False | Lock

type period_freq = Unset | Set of float * float

type periodic = {
    call: string;
    fname: string;
    period_freq: period_freq;
    delay: int option;
    start: string option;
    stop: string option;
    autorun: autorun
  }

let parse_periodic = fun attribs ->
  let get = fun x -> OT.assoc_opt x attribs in
  let geti = fun x ->  OT.assoc_opt_int x attribs in
  let call = List.find (fun a -> a = "fun") (fst (List.split attribs)) in
  let call_regexp = Str.regexp "\\([a-zA-Z_][a-zA-Z0-9_]*\\)\\(.*\\)" in
  let fname =
    if Str.string_match call_regexp call 0 then
      let fname = Str.matched_group 1 call and args = Str.matched_group 2 call in
      if args = "" || Str.string_match (Str.regexp "(.*)") args 0 then fname
      else failwith ("Module.parse_periodic: invalid function call: " ^ call)
    else failwith ("Module.parse_periodic: invalid function name: " ^ call) in
  let period_freq = match get "period", get "freq" with
  | None, None -> Unset
  | None, Some f -> let f = float_of_string f in Set (1. /. f, f)
  | Some p, None -> let p = float_of_string p in Set (p, 1. /. p)
  | Some p, Some _ ->
      Printf.eprintf "Warning: both period and freq are defined ";
      Printf.eprintf "but only period is used for function %s\n%!" fname;
      let p = float_of_string p in Set (p, 1. /. p) in
  { call; fname; period_freq; delay = geti "delay";
    start = get "start"; stop = get "stop";
    autorun = match get "autorun" with
    | None -> Lock
    | Some "TRUE" | Some "true" -> True
    | Some "FALSE" | Some "false" -> False
    | Some "LOCK" | Some "lock" -> Lock
    | Some _ -> failwith "Module.parse_periodic: unreachable" }

let fprint_period_freq = fun ch max_freq p ->
  let period, freq = match p.period_freq with
  | Unset -> 1. /. max_freq, max_freq
  | Set (p, f) -> p, f in
  let cap_fname = String.uppercase p.fname in (* TODO: deprecated function *)
  Printf.fprintf ch "#define %s_PERIOD %f\n" cap_fname period;
  Printf.fprintf ch "#define %s_FREQ %f\n" cap_fname freq

let status_name = fun mod_name p -> mod_name ^ "_" ^ p.fname ^ "_status"

let fprint_status = fun ch mod_name p ->
  match p.autorun with
  | True | False ->
      Printf.fprintf ch "EXTERN_MODULES uint8_t %s;\n" (status_name mod_name p)
  | Lock -> ()

let fprint_periodic_init = fun ch mod_name p ->
  match p.autorun with
  | True -> Printf.fprintf ch "%s = %s;" (status_name mod_name p) "MODULES_START"
  | False -> Printf.fprintf ch "%s = %s;" (status_name mod_name p) "MODULES_IDLE"
  | Lock -> ()

let fprint_init = fun ch init -> Printf.fprintf ch "%s;\n" init

type event = { ev: string; handlers: string list }

let make_event = fun f handlers ->
  { ev = f;
    handlers = List.map
      (function
        | Xml.Element ("handler", [("fun", f)], []) -> f
        | _ -> failwith "Module.make_event: unreachable"
      ) handlers }

let fprint_event = fun ch e -> Printf.fprintf ch "%s;\n" e.ev

type datalink = { message: string; func: string }

let fprint_datalink = fun ch d ->
  Printf.fprintf ch "(msg_id == DL_%s) { %s; }\n" d.message d.func

type t = {
    name: string;
    dir: string option;
    task: string option;
    path: string;
    doc: Xml.xml;
    requires: string list;
    conflicts: string list;
    provides: string list;
    autoloads: string list;
    settings: Settings.t list;
    headers: file list;
    inits: string list;
    periodics: periodic list;
    events: event list;
    datalinks: datalink list;
    makefile: makefile list;
    xml: Xml.xml
  }

let empty =
  { name = ""; dir = None; task = None; path = ""; doc = Xml.Element ("doc", [], []);
    requires = []; conflicts = []; provides = []; autoloads = []; settings = [];
    headers = []; inits = []; periodics = []; events = []; datalinks = [];
    makefile = []; xml = Xml.Element ("module", [], []) }

let parse_module_list = Str.split (Str.regexp "[ \t]*,[ \t]*")

let rec parse_xml m = function
  | Xml.Element ("module", attribs, children) as xml ->
      let name = find_name attribs
      and dir = OT.assoc_opt "dir" attribs
      and task = OT.assoc_opt "task" attribs in
      List.fold_left parse_xml { m with name; dir; task; xml } children
  | Xml.Element ("doc", _, _) as xml -> { m with doc = xml }
  (*| Xml.Element ("settings_file", [("name", name)], files) -> m (* TODO : remove unused *)*)
  | Xml.Element ("settings", _, _) as xml ->
      { m with settings = Settings.from_xml xml :: m.settings }
  | Xml.Element ("depends", _, [Xml.PCData depends]) ->
      { m with requires = parse_module_list depends }
  | Xml.Element ("conflicts", _, [Xml.PCData conflicts]) ->
      { m with conflicts = parse_module_list conflicts }
  | Xml.Element ("provides", _, [Xml.PCData provides]) ->
      { m with provides = parse_module_list provides }
  | Xml.Element ("autoload", attribs, []) ->
      let name = find_name attribs
      and atype = OT.assoc_opt "type" attribs in
      let module_name = match atype with None -> name | Some t -> name ^ "_" ^ t in (* CHECK *)
      { m with autoloads = module_name :: m.autoloads }
  | Xml.Element ("header", [], files) ->
      { m with headers =
        List.fold_left (fun acc f -> parse_file f :: acc) m.headers files }
  | Xml.Element ("init", [("fun", f)], []) -> { m with inits = f :: m.inits }
  | Xml.Element ("periodic", attribs, []) ->
      { m with periodics = parse_periodic attribs :: m.periodics }
  | Xml.Element ("event", [("fun", f)], handlers) ->
      { m with events = make_event f handlers :: m.events }
  | Xml.Element ("datalink", [("message", message); ("fun", func)], [])
  | Xml.Element ("datalink", [("fun", func); ("message", message)], []) ->
      { m with datalinks = { message; func } :: m.datalinks }
  | Xml.Element ("makefile", _, _) as xml ->
      { m with makefile = parse_makefile empty_makefile xml :: m.makefile }
  | _ -> failwith "Module.parse_xml: unreachable"

let from_xml = parse_xml empty

(** search and parse a module xml file and return a Module.t *)
let from_module_name = fun name mtype ->
  (* concat module type if needed *)
  let name = match mtype with Some t -> name ^ "_" ^ t | None -> name in
  (* determine if name already have an extension *)
  let name = if Filename.check_suffix name ".xml" then name else name ^ ".xml" in
  (* determine if name is implicit
   * if not, search for absolute name in search path
   * may raise Not_found if no file found *)
  let name =
    if Filename.is_implicit name then begin
      if Sys.file_exists name then name else raise Not_found
    end
    else begin
      let rec find_abs = function
        | [] -> raise Not_found
        | b::bl ->
            let full_name = Filename.concat b name in
            if Sys.file_exists full_name then full_name else find_abs bl
      in
      find_abs Env.modules_paths
    end in
  from_xml (ExtXml.parse_file name)


(** move to generators *)
let fprint_headers = fun ch m ->
  let dirname = match m.dir with None -> m.name | Some d -> d in
  List.iter
    (fun h ->
      let dir = match h.directory with None -> dirname | Some d -> d in
      Printf.fprintf ch "#include \"%s/%s\"\n" dir h.filename
  (* TODO: Filename.concat? *)
    ) m.headers
