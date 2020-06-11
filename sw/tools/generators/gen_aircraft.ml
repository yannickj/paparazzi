(*
 * Copyright (C) 2003-2009 Pascal Brisset, Antoine Drouin, ENAC
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
 * Main generator tool
 *)


open Printf
module U = Unix

module GC = Gen_common
module Af = Airframe
module AfT = Airframe.Target
module AfF = Airframe.Firmware

let (//) = Filename.concat

let paparazzi_conf = Env.paparazzi_home // "conf"
let default_conf_xml = paparazzi_conf // "conf.xml"

let airframe_h = "airframe.h"
let flight_plan_h = "flight_plan.h"
let flight_plan_dump = "flight_plan.xml"
let radio_h = "radio.h"
let periodic_h = "periodic_telemetry.h"
let modules_h = "modules.h"
let default_periodic_freq = 60
let default_modules_freq = 60

let get_string_opt = fun x -> match x with Some s -> s | None -> ""

(* type of loading (user, auto) *)
type load_type = UserLoad | AutoLoad | Unloaded

type target_conf = {
  configures: Module.configure list; (* configure variables *)
  configures_default: Module.configure list; (* default configure options *)
  defines: Module.define list; (* define flags *)
  firmware_name: string;
  board_type: string;
  modules: (load_type * Module.t) list; (* list of modules *)
  autopilot: Autopilot.t option; (* autopilot file if any *)
}

(* init structure *)
let init_target_conf = fun firmware_name board_type ->
  { configures = []; configures_default = []; defines = [];
    firmware_name; board_type; modules = []; autopilot = None }

let string_of_load = fun lt ->
  match lt with UserLoad -> "USER" | AutoLoad -> "AUTO" | Unloaded -> "UNLOAD"

(* add a module if compatible with target and firmware
 * and its autoloaded modules to a conf, return final conf *)
let rec target_conf_add_module = fun conf target firmware name mtype load_type ->
  printf "ADD MODULE (%s) %s %s %s\n" (string_of_load load_type) name target firmware;
  let m = Module.from_module_name name mtype in
  (* add autoloaded modules *)
  let conf = List.fold_left (fun c autoload ->
      target_conf_add_module c target firmware autoload.Module.aname autoload.Module.atype AutoLoad
    ) conf m.Module.autoloads in
  (* check compatibility with target *)
  if Module.check_loading target firmware m then
    (* add configures and defines to conf if needed *)
    { conf with
      configures = List.fold_left (fun cm mk ->
          if Module.check_mk target firmware mk then
            List.fold_left (fun cmk c ->
                if not (c.Module.cvalue = None) then cmk @ [c]
                else cmk
              ) cm mk.Module.configures
          else
            cm) conf.configures  m.Module.makefiles;
      configures_default = List.fold_left (fun cm mk ->
          if Module.check_mk target firmware mk then
            List.fold_left (fun cmk c ->
                if not (c.Module.default = None) then cmk @ [c]
                else cmk
              ) cm mk.Module.configures
          else
            cm) conf.configures  m.Module.makefiles;
      defines = List.fold_left (fun dm mk ->
          if Module.check_mk target firmware mk then dm @ mk.Module.defines else dm
        ) conf.defines  m.Module.makefiles;
      modules = conf.modules @ [load_type, m] }
  else begin
    printf "Unloading %s\n" name;
    (* add "unloaded" module for reference *)
    { conf with modules = conf.modules @ [(Unloaded, m)] } end


(* configuration sorted by target name: (string, target_conf) *)
let config_by_target = Hashtbl.create 5

(* sort element of an airframe type by target *)
let sort_airframe_by_target = fun airframe ->
  match airframe with
  | None -> ()
  | Some a ->
    (* build a list of pairs (target, firmware) *)
    let l = List.fold_left (fun lf f ->
        List.fold_left (fun lt t ->
            lt @ [(t, f)]) lf f.AfF.targets
      ) [] a.Af.firmwares in
    (* iterate on each target *)
    List.iter (fun (t, f) ->
        let name = t.AfT.name in (* target name *)
        Printf.printf "Handling target: %s\n%!" name;
        if Hashtbl.mem config_by_target name then
          failwith "[Error] Gen_airframe: two targets with the same name";
        (* init and add configure/define from airframe *)
        let conf = init_target_conf f.AfF.name t.AfT.board in
        let conf = { conf with
                     configures = t.AfT.configures @ f.AfF.configures;
                     defines = t.AfT.defines @ f.AfF.defines } in
        (* iter on modules in target *)
        let conf = List.fold_left (fun c m_af ->
            let c = { c with
                      configures = c.configures @ m_af.Module.configures;
                      defines = c.defines @ m_af.Module.defines } in
            target_conf_add_module c name f.AfF.name m_af.Module.name m_af.Module.mtype UserLoad
          ) conf t.AfT.modules in
        (* iter on modules in firmwares *)
        let conf = List.fold_left (fun c m_af ->
            let c = { c with
                      configures = c.configures @ m_af.Module.configures;
                      defines = c.defines @ m_af.Module.defines } in
            target_conf_add_module c name f.AfF.name m_af.Module.name m_af.Module.mtype UserLoad
          ) conf f.AfF.modules in
        Hashtbl.add config_by_target name conf
      ) l

let mkdir = fun d ->
  assert (Sys.command (sprintf "mkdir -p %s" d) = 0)

(** Raises a Failure if an ID or a NAME appears twice in the conf *)
let check_unique_id_and_name = fun conf conf_xml ->
  let ids = Hashtbl.create 5 and names = Hashtbl.create 5 in
  ExtXml.iter_tag "aircraft"
    (fun x ->
      let id = ExtXml.attrib x "ac_id"
      and name = ExtXml.attrib x "name" in
      if Hashtbl.mem ids id then begin
        let other_name = Hashtbl.find ids id in
        failwith (sprintf "Error: A/C Id '%s' duplicated in %s (%s and %s)" id conf_xml name other_name)
      end;
      if Hashtbl.mem names name then begin
        let other_id = Hashtbl.find names name in
        failwith (sprintf "Error: A/C name '%s' duplicated in %s (ids %s and %s)" name conf_xml id other_id)
      end;
      Hashtbl.add ids id name;
      Hashtbl.add names name id
    ) conf


let is_older = fun target_file dep_files ->
  not (Sys.file_exists target_file) ||
    let target_file_time = (U.stat target_file).U.st_mtime in
    let rec loop = function
      | [] -> false
      | f :: fs -> target_file_time < (U.stat f).U.st_mtime || loop fs in
    loop dep_files

let make_element = fun t a c -> Xml.Element (t,a,c)

(** Extract a configuration element from aircraft config,
 *  returns a tuple with absolute file path and element object
 * 
 * [bool -> Xml.xml -> string -> (Xml.xml -> a') -> (string * a' option)]
 *)
let get_config_element = fun flag ac_xml elt f ->
  if flag then None (* generation is not requested *)
  else
    (* try *) (* TODO: uncomment? *)
      let file = Xml.attrib ac_xml elt in
      let abs_file = paparazzi_conf // file in
      Some (f abs_file)
    (* with Xml.No_attribute _ -> None (\* no attribute elt in conf file *\) *)

(** Generate a configuration element
 *  Also check dependencies
 *
 * [a' -> (a' -> unit) -> (string * string list) list -> unit]
 *)
let generate_config_element = fun elt f dep_list ->
  if List.exists (fun (file, dep) -> is_older file dep) dep_list then f elt
  else ()

(******************************* MAIN ****************************************)
let () =
  let ac_name = ref None
  and target_name = ref None
  and conf_xml = ref default_conf_xml
  and gen_af = ref false
  and gen_fp = ref false
  and gen_set = ref false
  and gen_rc = ref false
  and gen_tl = ref false
  and modules_freq = ref default_modules_freq
  and tl_freq = ref default_periodic_freq in

  let options =
    [ "-name", Arg.String (fun x -> ac_name := Some x), " Aircraft name (mandatory)";
      "-target", Arg.String (fun x -> target_name := Some x), " Target to build (mandatory)";
      "-conf", Arg.String (fun x -> conf_xml := x), (sprintf " Configuration file (default '%s')" default_conf_xml);
      "-airframe", Arg.Set gen_af, " Generate airframe file";
      "-flight_plan", Arg.Set gen_fp, " Generate flight plan file";
      "-settings", Arg.Set gen_set, " Generate settings file";
      "-radio", Arg.Set gen_set, " Generate radio file";
      "-telemetry", Arg.Set gen_tl, " Generate telemetry file";
      "-periodic_freq", Arg.Int (fun x -> tl_freq := x), (sprintf " Periodic telemetry frequency (default %d)" default_periodic_freq);
      "-modules_freq", Arg.Int (fun x -> modules_freq := x), (sprintf " Modules frequency (default %d)" default_modules_freq);
      ] in

  Arg.parse
    options
    (fun x -> Printf.fprintf stderr "%s: Warning: Don't do anything with '%s' argument\n" Sys.argv.(0) x)
    "Usage: ";

  (* check aircraft and target options *)
  let aircraft =
    match !ac_name with
    | None -> failwith "An aircraft name is mandatory"
    | Some ac -> ac
  in
  let target =
    match !target_name with
    | None -> failwith "A target name is mandatory"
    | Some t -> t
  in
  Printf.printf "Target: %s\n%!" target;

  try
    let conf = ExtXml.parse_file !conf_xml in
    check_unique_id_and_name conf !conf_xml;
    let aircraft_xml =
      try
        ExtXml.child conf ~select:(fun x -> Xml.attrib x "name" = aircraft) "aircraft"
      with
        Not_found -> failwith (sprintf "Aircraft '%s' not found in '%s'" aircraft !conf_xml)
    in

    let value = fun attrib -> ExtXml.attrib aircraft_xml attrib in

    (* Prepare building folders *)
    let aircraft_dir = Env.paparazzi_home // "var" // "aircrafts" // aircraft in
    let aircraft_conf_dir = aircraft_dir // "conf" in
    let aircraft_gen_dir = aircraft_dir // target // "generated" in
    mkdir (Env.paparazzi_home // "var");
    mkdir (Env.paparazzi_home // "var" // "aircrafts");
    mkdir aircraft_dir;
    mkdir (aircraft_dir // target);
    mkdir aircraft_conf_dir;
    mkdir aircraft_gen_dir;
    mkdir (aircraft_conf_dir // "airframes");
    mkdir (aircraft_conf_dir // "flight_plans");
    mkdir (aircraft_conf_dir // "radios");
    mkdir (aircraft_conf_dir // "settings");
    mkdir (aircraft_conf_dir // "telemetry");

    Printf.printf "%s\n%b\n%!" (Xml.to_string_fmt aircraft_xml) !gen_rc;

    (* Parse file if needed *)
    Printf.printf "Parsing airframe...%!";
    let airframe = get_config_element !gen_af aircraft_xml "airframe" Airframe.from_file in
    Printf.printf "done.\nSorting...%!";
    sort_airframe_by_target airframe;
    Printf.printf " done.\nParsing flight plan...%!";
    let flight_plan = get_config_element !gen_fp aircraft_xml "flight_plan" Flight_plan.from_file in
    Printf.printf " done\nExtracting modules from FP...%!";
    begin match flight_plan with
      | None -> ()
      | Some fp ->
        Printf.printf "\nModules from FP %s: " fp.Flight_plan.filename;
        List.iter (fun (m : Module.config) -> Printf.printf "%s " m.Module.name) fp.Flight_plan.modules; Printf.printf "\n%!";
        Hashtbl.iter (fun target conf ->
          let conf = List.fold_left (fun c m ->
              let c = { c with
                        configures = c.configures @ m.Module.configures;
                        defines = c.defines @ m.Module.defines } in
              target_conf_add_module c target "" m.Module.name m.Module.mtype UserLoad
          ) conf fp.Flight_plan.modules in
          Hashtbl.replace config_by_target target conf
        ) config_by_target
    end;
    (* TODO : same with autopilot *)
    Printf.printf " done\nParsing radio...%!";
    let radio = get_config_element !gen_rc aircraft_xml "radio" Radio.from_file in
    Printf.printf " done\nParsing telemetry...%!";
    let telemetry = get_config_element !gen_tl aircraft_xml "telemetry" Telemetry.from_file in
    Printf.printf " done\n%!";

    (* TODO? filter duplicates: seems it's done in target_conf_add_module *)
    (* TODO resolve modules dep *)

    (* Generate output files *)
    Printf.printf "Dumping aircraft header...%!";
    let abs_airframe_h = aircraft_gen_dir // airframe_h in
    begin match airframe with
      | None -> ()
      | Some airframe ->
        generate_config_element airframe
          (fun e ->
             Gen_airframe.generate
               e (value "ac_id") (get_string_opt !ac_name)
               "0x42" airframe.Airframe.filename abs_airframe_h)
          (* TODO compute correct MD5SUM *)
          [ (abs_airframe_h, [airframe.Airframe.filename]) ] end;
    (* TODO add dep in included files *)

    Printf.printf " done\nDumping flight plan XML and header...%!";
    let abs_flight_plan_h = aircraft_gen_dir // flight_plan_h in
    let abs_flight_plan_dump = aircraft_gen_dir // flight_plan_dump in
    begin match flight_plan with
      | None -> ()
      | Some flight_plan ->
        generate_config_element flight_plan
          (fun e ->
             Gen_flight_plan.generate
               e flight_plan.Flight_plan.filename abs_flight_plan_h)
          [ (abs_flight_plan_h, [flight_plan.Flight_plan.filename]) ];
        generate_config_element flight_plan
          (fun e ->
             Gen_flight_plan.generate
               e ~dump:true flight_plan.Flight_plan.filename abs_flight_plan_dump)
          [ (abs_flight_plan_dump, [flight_plan.Flight_plan.filename]) ]
    end;

    Printf.printf " done\nDumping radio header...%!";
    let abs_radio_h = aircraft_gen_dir // radio_h in
    begin match radio with
      | None -> ()
      | Some radio ->
        generate_config_element radio
          (fun e -> Gen_radio.generate e radio.Radio.filename abs_radio_h)
          [ (abs_radio_h, [radio.Radio.filename]) ] end;

    Printf.printf " done\nDumping telemetry header...%!";
    let abs_telemetry_h = aircraft_gen_dir // periodic_h in
    begin match telemetry with
      | None -> ()
      | Some telemetry ->
        generate_config_element telemetry
          (fun e -> Gen_periodic.generate e !tl_freq abs_telemetry_h)
          [ (abs_telemetry_h, [telemetry.Telemetry.filename]) ] end;
    Printf.printf " done\n%!";

    Printf.printf "target->firmware: ";
    Hashtbl.iter (fun tgt cfg -> Printf.printf "(%s -> %s) " tgt cfg.firmware_name) config_by_target;
    Printf.printf "\n%!";

    let config = Hashtbl.find config_by_target target in
    let modules = config.modules in
    Printf.printf "Modules: ";
    List.iter (fun (_, m) -> Printf.printf "%s " m.Module.name) modules;
    Printf.printf "\n%!";
    (* normal settings *)
    let settings = try Env.filter_settings (value "settings") with _ -> "" in
    let settings_files = Str.split (Str.regexp " ") settings in
    let settings = List.map
        (fun f -> Settings.from_file (paparazzi_conf // f)) settings_files in
    (* modules settings *)
    let settings_modules =
      try Env.filter_settings (value "settings_modules")
      with _ -> "" in
    let settings_modules_files = Str.split (Str.regexp " ") settings_modules in
    let settings_modules = List.fold_left
        (fun acc f ->
           let mods = Module.from_file (paparazzi_conf // f) in
           acc @ mods.Module.settings
        ) [] settings_modules_files in
    Printf.printf " done\nDumping modules header...%!";
    (*List.iter (fun (t, m) ->
      match t with UserLoad -> printf "USER %s\n" m.Module.filename
      | AutoLoad -> printf "AUTO %s\n" m.Module.filename
      | Unloaded -> printf "UNLOAD %s\n" m.Module.filename
    ) modules;*)
    let abs_modules_h = aircraft_gen_dir // modules_h in
    generate_config_element (List.fold_left (fun l (t, m) -> if t <> Unloaded then l @ [m] else l) [] modules)
      (fun e -> Gen_modules.generate e !modules_freq "" abs_modules_h)
      [ abs_modules_h, List.map (fun (_, m) -> m.Module.filename) modules ];
    Printf.printf " done\n%!";
    

    (* TODO: update aircraft with all above settings *)
    (* finally, concat all settings and filter on target *)
    let settings = List.fold_left (fun acc s ->
        if Gen_common.test_targets target
            (Gen_common.targets_of_string s.Settings.target)
        then acc @ [s] else acc) [] (settings @ settings_modules) in
    let settings = if List.length settings = 0 then
      begin
        Printf.eprintf "\nInfo: No 'settings' attribute specified for A/C '%s', using 'settings/dummy.xml'\n\n%!" aircraft;
        [Settings.from_file "settings/dummy.xml"]
      end
      else settings
    in
    List.iter (fun s -> printf "Set: %s %s\n" s.Settings.filename (Xml.to_string s.Settings.xml)) settings;
    (*Printf.printf "Settings: ";
    List.iter
      (fun s -> Printf.printf "%s(%s) "
          (match s.Settings.name with None -> s.Settings.filename | Some s -> s)
          (match s.Settings.target with None -> "all tagets" | Some t -> t))
      settings;
    Printf.printf "\n%!";*)

    (** Expands the configuration of the A/C into one single file *)
    let conf_aircraft = Env.expand_ac_xml aircraft_xml in
    let configuration =
      make_element
        "configuration" []
        [ make_element "conf" [] [conf_aircraft]; PprzLink.messages_xml () ] in
    let conf_aircraft_file = aircraft_conf_dir // "conf_aircraft.xml" in
    let f = open_out conf_aircraft_file in
    Printf.fprintf f "%s\n" (ExtXml.to_string_fmt configuration);
    close_out f;

    (** Computes and store a signature of the configuration *)
    let md5sum = Digest.to_hex (Digest.file conf_aircraft_file) in
    let md5sum_file = aircraft_conf_dir // "aircraft.md5" in
    (* Store only if different from previous one *)
    if not (Sys.file_exists md5sum_file
            && md5sum = input_line (open_in md5sum_file)) then begin
      let f = open_out md5sum_file in
      Printf.fprintf f "%s\n" md5sum;
      close_out f;

      (** Save the configuration for future use *)
      let d = U.localtime (U.gettimeofday ()) in
      let filename = sprintf "%02d_%02d_%02d__%02d_%02d_%02d_%s_%s.conf"
          (d.U.tm_year mod 100) (d.U.tm_mon+1) (d.U.tm_mday)
          (d.U.tm_hour) (d.U.tm_min) (d.U.tm_sec)
          md5sum aircraft in
      let d = Env.paparazzi_home // "var" // "conf" in
      mkdir d;
      let f = open_out (d // filename) in
      Printf.fprintf f "%s\n" (ExtXml.to_string_fmt configuration);
      close_out f end;

(**

    let airframe_dir = Filename.dirname airframe_file in
    let var_airframe_dir = aircraft_conf_dir // airframe_dir in
    mkdir var_airframe_dir;
    assert (Sys.command (sprintf "cp %s %s" (paparazzi_conf // airframe_file) var_airframe_dir) = 0);

    (** Calls the Makefile with target and options *)
    let make = fun target options ->
      let c = sprintf "make -f Makefile.ac AIRCRAFT=%s AC_ID=%s AIRFRAME_XML=%s TELEMETRY=%s SETTINGS=\"%s\" MD5SUM=\"%s\" %s %s" aircraft (value "ac_id") airframe_file (value "telemetry") settings md5sum options target in
      begin (** Quiet is speficied in the Makefile *)
        try if Sys.getenv "Q" <> "@" then raise Not_found with
            Not_found -> prerr_endline c
      end;
      let returned_code = Sys.command c in
      if returned_code <> 0 then
        exit returned_code in

    (** Calls the makefile if the optional attribute is available *)
    let make_opt = fun target var attr ->
      try
        let value = Xml.attrib aircraft_xml attr in
        make target (sprintf "%s=%s" var value)
      with Xml.No_attribute _ -> () in
*)

    let temp_makefile_ac = Filename.temp_file "Makefile.ac" "tmp" in
    begin match airframe, flight_plan with
    | Some af, Some fp ->
      Gen_makefile.generate_makefile af.Airframe.name af fp temp_makefile_ac
    | _ -> Printf.eprintf "Missing airframe or flight_plan" end;

    (* Create Makefile.ac only if needed *)
    let makefile_ac = aircraft_dir // "Makefile.ac" in
    match airframe with
    | None -> ()
    | Some airframe ->
      let module_files = List.map (fun (_, m) -> m.Module.filename) modules in
      if is_older makefile_ac (airframe.Airframe.filename :: module_files)
      then
        assert(Sys.command (sprintf "mv %s %s" temp_makefile_ac makefile_ac) = 0)
      ;

(*
    (* Get TARGET env, needed to build modules.h according to the target *)
    let t = try Printf.sprintf "TARGET=%s" (Sys.getenv "TARGET") with _ -> "" in
    (* Get FLIGHT_PLAN attribute, needed to build modules.h as well FIXME *)
    let t = t ^ try Printf.sprintf " FLIGHT_PLAN=%s" (Xml.attrib aircraft_xml "flight_plan") with _ -> "" in
    make_opt "radio_ac_h" "RADIO" "radio";
    make_opt "flight_plan_ac_h" "FLIGHT_PLAN" "flight_plan";
    make "all_ac_h" t
    *)
  with Failure f ->
    prerr_endline f;
    exit 1
