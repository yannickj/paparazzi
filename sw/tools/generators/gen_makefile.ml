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
 * Generate makefile from the aircraft configuration
 *)

open Printf
module U = Unix

open Gen_common

let (//) = Filename.concat


let configure_xml2mk = fun ?(default_configure=false) f xml ->
  (* all makefiles variables are forced to uppercase *)
  let name = Compat.uppercase_ascii (ExtXml.attrib xml "name")
  and value = ExtXml.attrib_or_default xml "value" ""
  and default = ExtXml.attrib_or_default xml "default" ""
  and case = ExtXml.attrib_or_default xml "case" "" in
  (* either print the default or the normal configure variable *)
  if default_configure then begin
    (* Only print variable if default is set but not value *)
    if String.length default > 0 && String.length value = 0 then
      fprintf f "%s ?= %s\n" name default;
    (* also providing lower and upper case version on request *)
    if Str.string_match (Str.regexp ".*lower.*") case 0 then
      fprintf f "%s_LOWER = $(shell echo $(%s) | tr A-Z a-z)\n" name name;
    if Str.string_match (Str.regexp ".*upper.*") case 0 then
      fprintf f "%s_UPPER = $(shell echo $(%s) | tr a-z A-Z)\n" name name
  end
  else
    (* Only print variable if value is not empty *)
    if String.length value > 0 then
      fprintf f "%s = %s\n" name value;
    (* Or if only the name is given (unset a variable *)
    if String.length value = 0 && String.length default = 0 && String.length case = 0 then
      fprintf f "%s =\n" name

let include_xml2mk = fun f ?(target="$(TARGET)") ?(vpath=None) xml ->
  let name = Xml.attrib xml "name"
  and path = match vpath with Some vp -> vp ^ "/" | None -> "" in
  let flag = sprintf "%s.CFLAGS += -I%s%s" target path name in
  try
    (* TODO: add condition in xml syntax ? *)
    let cond = Xml.attrib xml "cond" in
    fprintf f "%s\n%s\nendif\n" cond flag
  with Xml.No_attribute _ -> fprintf f "%s\n" flag

let flag_xml2mk = fun f ?(target="$(TARGET)") xml ->
  let name = Xml.attrib xml "name"
  and value = Xml.attrib xml "value" in
  let flag = sprintf "%s.%s += -%s" target name value in
  try
    (* TODO: add condition in xml syntax ? *)
    let cond = Xml.attrib xml "cond" in
    fprintf f "%s\n%s\nendif\n" cond flag
  with Xml.No_attribute _ -> fprintf f "%s\n" flag

let define_xml2mk = fun f ?(target="$(TARGET)") xml ->
  let name = Xml.attrib xml "name"
  and value = try Some (Xml.attrib xml "value") with _ -> None in
  let flag_type = fun s ->
    match ExtXml.attrib_or_default xml "type" "raw", value with
    | "string", Some v -> "=\\\""^v^"\\\""
    | _, Some v -> "="^v
    | _, _ -> ""
  in
  let flag = sprintf "%s.CFLAGS += -D%s%s" target name (flag_type value) in
  try
    (* TODO: add condition in xml syntax ? *)
    let cond = Xml.attrib xml "cond" in
    fprintf f "%s\n%s\nendif\n" cond flag
  with Xml.No_attribute _ -> fprintf f "%s\n" flag

let raw_xml2mk = fun f name xml ->
  match Xml.children xml with
  | [Xml.PCData s] -> fprintf f "%s\n" s
  | _ -> eprintf "Warning: wrong makefile section in '%s': %s\n"
        name (Xml.to_string_fmt xml)

let file_xml2mk = fun f ?(arch = false) dir_name target xml ->
  let name = Xml.attrib xml "name" in
  let dir_name = ExtXml.attrib_or_default xml "dir" ("$(" ^ dir_name ^ ")") in
  let cond, cond_end = try "\n"^(Xml.attrib xml "cond")^"\n", "\nendif" with Xml.No_attribute _ -> "", "" in
  let fmt =
    if arch then format_of_string "%s%s.srcs += arch/$(ARCH)/%s/%s%s\n"
    else format_of_string "%s%s.srcs += %s/%s%s\n" in
  fprintf f fmt cond target dir_name name cond_end

(* only print the configuration flags for a module
 * 'raw' section are not handled here
 *)
let module_configure_xml2mk = fun ?(default_configure=false) f target firmware m ->
  (* print global config flags *)
  List.iter (fun flag ->
    match Compat.lowercase_ascii (Xml.tag flag) with
    | "configure" -> configure_xml2mk ~default_configure f flag
    | _ -> ()) m.param;
  (* Look for makefile section *)
  ExtXml.iter_tag "makefile"
    (fun section ->
      (* Look for defines, flags, files, ... if target is matching *)
      let section =
        let targets = Gen_common.targets_of_field section Env.default_module_targets in
        if Gen_common.test_targets target targets then section else Xml.Element ("makefile", [], [])
      in
      (* keep section if firmware is also matching or not speficied *)
      let section = begin
        try
          if Xml.attrib section "firmware" = firmware then section
          else Xml.Element ("makefile", [], [])
        with _ -> section end
      in
      Xml.iter
      (fun field ->
          match Compat.lowercase_ascii (Xml.tag field) with
          | "configure" -> configure_xml2mk ~default_configure f field
          | _ -> ()
        ) section
    ) m.xml

(* module files and flags except configuration flags *)
let module_xml2mk = fun f target firmware m ->
  let name = ExtXml.attrib m.xml "name" in
  let dir = try Xml.attrib m.xml "dir" with Xml.No_attribute _ -> name in
  let dir_name = Compat.uppercase_ascii dir ^ "_DIR" in
  (* print global flags as compilation defines and flags *)
  fprintf f "\n# makefile for module %s in modules/%s\n" name dir;
  List.iter (fun flag ->
    match Compat.lowercase_ascii (Xml.tag flag) with
    | "define" -> define_xml2mk f ~target flag
    | _ -> ()) m.param;
  (* Look for makefile section *)
  ExtXml.iter_tag "makefile"
    (fun section ->
      (* Look for defines, flags, files, ... if target is matching *)
      let section =
        let targets = Gen_common.targets_of_field section Env.default_module_targets in
        if Gen_common.test_targets target targets then section else Xml.Element ("makefile", [], [])
      in
      (* keep section if firmware is also matching or not speficied *)
      let section = begin
        try
          if Xml.attrib section "firmware" = firmware then section
          else Xml.Element ("makefile", [], [])
        with _ -> section end
      in
      (* add condition if need *)
      let cond = try Some (Xml.attrib section "cond") with _ -> None in
      let _ = match cond with Some c -> fprintf f "%s\n" c | None -> () in
      Xml.iter
      (fun field ->
          match Compat.lowercase_ascii (Xml.tag field) with
          | "define" -> define_xml2mk f ~target field
          | "include" -> include_xml2mk f ~target ~vpath:m.vpath field
          | "flag" -> flag_xml2mk f ~target field
          | "file" -> file_xml2mk f dir_name target field
          | "file_arch" -> file_xml2mk f ~arch:true dir_name target field
          | "raw" -> raw_xml2mk f name field
          | _ -> ()
      ) section;
      match cond with Some _ -> fprintf f "endif\n" | None -> ()
    ) m.xml

let modules_xml2mk = fun f target ac_id xml fp ->
  let modules = Gen_common.get_modules_of_config ~target ~verbose:true ac_id xml fp in
  (* print modules directories and includes for all targets *)
  fprintf f "\n# include modules directory for all targets\n";
  (* get dir list *)
  let dir_list = Gen_common.get_modules_dir modules in
  (** include modules directory for ALL targets, not just the defined ones **)
  fprintf f "$(TARGET).CFLAGS += -Imodules -Iarch/$(ARCH)/modules\n";
  List.iter
    (fun dir -> fprintf f "%s_DIR = modules/%s\n" (Compat.uppercase_ascii dir) dir
    ) dir_list;
  (* add vpath for external modules *)
  List.iter
    (fun m -> match m.vpath with
    | Some vp -> fprintf f "VPATH += %s\n" vp
    | _ -> ()
    ) modules;
  fprintf f "\n";
  modules

(** Firmware Children *)
let subsystem_xml2mk = fun f firmware s ->
  let name = ExtXml.attrib s "name"
  and s_type = try "_" ^ (Xml.attrib s "type") with Xml.No_attribute _ -> "" in
  fprintf f "\n# -subsystem: '%s'\n" name;
  let s_config, rest = ExtXml.partition_tag "configure" (Xml.children s) in
  let s_defines, _ = ExtXml.partition_tag "define" rest in
  (*List.iter (configure_xml2mk f) s_config;*)
  List.iter (fun def -> define_xml2mk f def) s_defines;
  (* include subsystem *) (* TODO test if file exists with the generator ? *)
  let s_name = name ^ s_type ^ ".makefile" in
  let s_dir = "CFG_" ^ Compat.uppercase_ascii (Xml.attrib firmware "name") in
  fprintf f "ifneq ($(strip $(wildcard $(%s)/%s)),)\n" s_dir s_name;
  fprintf f "\tinclude $(%s)/%s\n" s_dir s_name;
  fprintf f "else\n";
  fprintf f "\tinclude $(CFG_SHARED)/%s\n" s_name;
  fprintf f "endif\n"

let subsystem_configure_xml2mk = fun f s ->
  let s_config, _ = ExtXml.partition_tag "configure" (Xml.children s) in
  List.iter (configure_xml2mk f) s_config

(** if xml node valid module, do notihg, otherwise fall back to subsystem *)
let fallback_subsys_xml2mk = fun f global_targets firmware target xml ->
  try
    ignore(Gen_common.get_module xml global_targets)
  with Gen_common.Subsystem _file -> subsystem_xml2mk f firmware xml

let parse_firmware = fun makefile_ac ac_id ac_xml firmware fp ->
  let firmware_name = Xml.attrib firmware "name" in
  (* get the configures, targets, subsystems and defines for this firmware *)
  let config, rest = ExtXml.partition_tag "configure" (Xml.children firmware) in
  let targets, rest = ExtXml.partition_tag "target" rest in
  let mods, rest = ExtXml.partition_tag "module" rest in
  let subsystems, rest = ExtXml.partition_tag "subsystem" rest in
  let defines, _ = ExtXml.partition_tag "define" rest in
  (* iter on all targets *)
  List.iter (fun target ->
    (* get configures, defines and subsystems for this target *)
    let t_config, rest = ExtXml.partition_tag "configure" (Xml.children target) in
    let t_defines, rest = ExtXml.partition_tag "define" rest in
    let t_mods, rest = ExtXml.partition_tag "module" rest in
    let t_subsystems, _ = ExtXml.partition_tag "subsystem" rest in
    (* print makefile for this target *)
    let target_name = Xml.attrib target "name" in
    fprintf makefile_ac "\n###########\n# -target: '%s'\n" target_name;
    fprintf makefile_ac "ifeq ($(TARGET), %s)\n" target_name;
    let target_name = Xml.attrib target "name" in
    let modules = modules_xml2mk makefile_ac target_name ac_id ac_xml fp in
    begin (* Check for "processor" attribute *)
      try
        let proc = Xml.attrib target "processor" in
        fprintf makefile_ac "BOARD_PROCESSOR = %s\n" proc
      with Xml.No_attribute _ -> ()
    end;
    begin (* auto activation of generated autopilot if needed *)
      try
        let _ = Gen_common.get_autopilot_of_airframe ~target:target_name ac_xml in
        fprintf makefile_ac "USE_GENERATED_AUTOPILOT = TRUE\n";
      with Not_found -> ()
    end;
    List.iter (configure_xml2mk makefile_ac) config;
    List.iter (configure_xml2mk makefile_ac) t_config;
    List.iter (subsystem_configure_xml2mk makefile_ac) subsystems;
    List.iter (subsystem_configure_xml2mk makefile_ac) t_subsystems;
    List.iter (subsystem_configure_xml2mk makefile_ac) mods;
    List.iter (subsystem_configure_xml2mk makefile_ac) t_mods;
    List.iter (module_configure_xml2mk makefile_ac target_name firmware_name) modules; (* print normal configure from module xml *)
    fprintf makefile_ac "\ninclude $(PAPARAZZI_SRC)/conf/boards/%s.makefile\n" (Xml.attrib target "board");
    fprintf makefile_ac "include $(PAPARAZZI_SRC)/conf/firmwares/%s.makefile\n\n" (Xml.attrib firmware "name");
    List.iter (module_configure_xml2mk ~default_configure:true makefile_ac target_name firmware_name) modules; (* print default configure from module xml *)
    fprintf makefile_ac "\n";
    List.iter (fun def -> define_xml2mk makefile_ac def) defines;
    List.iter (fun def -> define_xml2mk makefile_ac def) t_defines;
    List.iter (module_xml2mk makefile_ac target_name firmware_name) modules;
    List.iter (fallback_subsys_xml2mk makefile_ac (Gen_common.Var "") firmware target_name) mods;
    List.iter (fallback_subsys_xml2mk makefile_ac (Gen_common.Var "") firmware target_name) t_mods;
    List.iter (subsystem_xml2mk makefile_ac firmware) t_subsystems;
    List.iter (subsystem_xml2mk makefile_ac firmware) subsystems;
    fprintf makefile_ac "\nendif # end of target '%s'\n\n" target_name
  ) targets


(** Search and dump the firmware section *)
let dump_firmware = fun f ac_id ac_xml firmware fp ->
  try
    fprintf f "\n####################################################\n";
    fprintf f   "# makefile firmware '%s'\n" (Xml.attrib firmware "name");
    fprintf f   "####################################################\n";
    parse_firmware f ac_id ac_xml firmware fp
  with Xml.No_attribute _ -> failwith "Warning: firmware name is undeclared"

let dump_firmware_sections = fun makefile_ac ac_id fp xml ->
  ExtXml.iter_tag "firmware"
    (fun tag -> dump_firmware makefile_ac ac_id xml tag fp) xml

(** Generate makefile configuration files *)
let generate_makefile = fun ac_id airframe flight_plan makefile_out ->
  let f = open_out makefile_out in
  fprintf f "# This file has been generated by gen_aircraft\n";
  fprintf f "# Version %s\n" (Env.get_paparazzi_version ());
  fprintf f "# Please DO NOT EDIT\n";
  fprintf f "AC_ID=%s\n" ac_id;

  (** Search and dump the firmware sections *)
  dump_firmware_sections f ac_id airframe.Airframe.xml flight_plan.Flight_plan.xml;

