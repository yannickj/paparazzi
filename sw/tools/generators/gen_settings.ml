(*
 * XML preprocessing for dynamic tuning
 *
 * Copyright (C) 2006 Pascal Brisset, Antoine Drouin
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 *)

(** Generates code for tuning parameters using RC sliders and datalink *)

open Printf
open Xml2h


let margin = ref 0
let step = 2
let tab = fun out -> fprintf out "%s" (String.make !margin ' ')
let right () = margin := !margin + step
let left () = margin := !margin - step

let lprintf = fun out f -> tab out; fprintf out f

let rec flatten = fun xml r ->
  if ExtXml.tag_is xml "dl_setting" then
    xml::r
  else
    match Xml.children xml with
        [] -> r
      | x::xs ->
        List.iter (fun y -> assert(ExtXml.tag_is y (Xml.tag x))) xs;
        List.fold_right flatten (x::xs) r


module StringSet = Set.Make(struct type t = string let compare = compare end)


let print_dl_settings = fun out settings settings_xml ->
  let settings_xml = flatten settings_xml [] in

  (** include  headers **)
  (*let modules = ref StringSet.empty in
  List.iter
    (fun s ->
      try
        modules := StringSet.add (ExtXml.attrib s "module") !modules
      with ExtXml.Error e -> ()
    )
    settings;*)

  lprintf out "\n";
  List.iter (fun s ->
    List.iter (fun h ->
      lprintf out "#include \"%s.h\"\n" h
    ) (Settings.get_headers s)
  ) settings;
  (*StringSet.iter (fun m -> lprintf out "#include \"%s.h\"\n" m) !modules;*)
  lprintf out "#include \"generated/modules.h\"\n";
  lprintf out "\n";

  (** Datalink knowing what settings mean **)
  Xml2h.define_out out "SETTINGS_NAMES" "{ \\";
  List.iter (fun b -> fprintf out " { \"%s\" }, \\\n" (ExtXml.attrib b "var")) settings_xml;
  lprintf out "};\n";

  Xml2h.define_out out "SETTINGS_NAMES_SHORT" "{ \\";
  List.iter (fun b ->
    let varname = Str.split (Str.regexp "[_.]+") (ExtXml.attrib b "var") in
    let shortname = List.fold_left (fun acc c ->
      try acc ^"_"^ (Str.first_chars c 3) with _ -> acc ^"_"^ c
    ) "" varname in
    let shorted = try String.sub shortname 1 16 with _ -> String.sub shortname 1 ((String.length shortname)-1) in
    fprintf out " \"%s\" , \\\n" shorted
  ) settings_xml;
  lprintf out "};\n";
  Xml2h.define_out out "NB_SETTING" (string_of_int (List.length settings_xml));

  (** Macro to call to set one variable *)
  lprintf out "#define DlSetting(_idx, _value) { \\\n";
  right ();
  lprintf out "switch (_idx) { \\\n";
  right ();
  let idx = ref 0 in
  List.iter
    (fun s ->
      let v = ExtXml.attrib s "var" in
      begin
        try
          let h = ExtXml.attrib s "handler" in
          begin
            try
              let m =  ExtXml.attrib s "module" in
              lprintf out "case %d: %s_%s( _value ); _value = %s; break;\\\n" !idx (Filename.basename m) h v
            with
                ExtXml.Error e -> prerr_endline (sprintf "Error: You need to specify the module attribute for setting %s to use the handler %s" v h); exit 1
          end;
        with
            ExtXml.Error e -> lprintf out "case %d: %s = _value; break;\\\n" !idx v
      end;
      incr idx
    )
    settings_xml;
  lprintf out "default: break;\\\n";
  left ();
  lprintf out "}\\\n";
  left ();
  lprintf out "}\n";
  let nb_values = !idx in

  (** Macro to call to downlink current values *)
  lprintf out "#define PeriodicSendDlValue(_trans, _dev) { \\\n";
  if nb_values > 0 then begin
    right ();
    lprintf out "static uint8_t i;\\\n";
    lprintf out "float var;\\\n";
    lprintf out "if (i >= %d) i = 0;\\\n" nb_values;
    let idx = ref 0 in
    lprintf out "switch (i) { \\\n";
    right ();
    List.iter
      (fun s ->
        let v = ExtXml.attrib s "var" in
        lprintf out "case %d: var = %s; break;\\\n" !idx v; incr idx)
      settings_xml;
    lprintf out "default: var = 0.; break;\\\n";
    left ();
    lprintf out "}\\\n";
    lprintf out "pprz_msg_send_DL_VALUE(_trans, _dev, AC_ID, &i, &var);\\\n";
    lprintf out "i++;\\\n";
    left ()
  end;
  lprintf out "}\n";

  (** Inline function to get a setting value *)
  lprintf out "static inline float settings_get_value(uint8_t i) {\n";
  right ();
  let idx = ref 0 in
  lprintf out "switch (i) {\n";
  right ();
  List.iter
    (fun s ->
      let v = ExtXml.attrib s "var" in
      lprintf out "case %d: return %s;\n" !idx v; incr idx)
    settings_xml;
  lprintf out "default: return 0.;\n";
  left ();
  lprintf out "}\n";
  left ();
  lprintf out "}\n"


let inttype = function
"bool" -> "uint8_t"
  | "int8" -> "int8_t"
  | "int16" -> "int16_t"
  | "int32" -> "int32_t"
  | "int64" -> "int64_t"
  | "uint8" -> "uint8_t"
  | "uint16" -> "uint16_t"
  | "uint32" -> "uint32_t"
  | "uint64" -> "uint64_t"
  | "float" -> "float"
  | "double" -> "double"
  | x -> failwith (sprintf "Gen_calib.inttype: unknown type '%s'" x)


(*
  Generate code for persistent settings
*)
let print_persistent_settings = fun out settings settings_xml ->
  let settings_xml = flatten settings_xml [] in
  let pers_settings =
    List.filter (fun x -> try let _ = Xml.attrib x "persistent" in true with _ -> false) settings_xml in
  (* structure declaration *)
  (*  if List.length pers_settings > 0 then begin *)
  lprintf out "\n/* Persistent Settings */\n";
  lprintf out "struct PersistentSettings {\n";
  right();
  let idx = ref 0 in
  List.iter
    (fun s ->
      let v = ExtXml.attrib s "var" in
      let t = try ExtXml.attrib s "type" with _ -> "float" in
      lprintf out "%s s_%d; /* %s */\n" (inttype t) !idx v; incr idx)
    pers_settings;
  left();
  lprintf out "};\n\n";
  lprintf out "extern struct PersistentSettings pers_settings;\n\n";
  (*  Inline function to store persistent settings *)
  idx := 0;
  lprintf out "static inline void persistent_settings_store( void ) {\n";
  right();
  List.iter
    (fun s ->
      let v = ExtXml.attrib s "var" in
      lprintf out "pers_settings.s_%d = %s;\n" !idx v; incr idx)
    pers_settings;
  left();
  lprintf out "}\n\n";
  (*  Inline function to load persistent settings *)
  idx := 0;
  lprintf out "static inline void persistent_settings_load( void ) {\n";
  right();
  List.iter
    (fun s ->
      let v = ExtXml.attrib s "var" in
      begin
        try
          let h = ExtXml.attrib s "handler" and
              m =  ExtXml.attrib s "module" in
          lprintf out "%s_%s( pers_settings.s_%d );\n"  (Filename.basename m) h !idx ;
        (*     lprintf out "%s = pers_settings.s_%d;\n" v !idx *) (* do we want to set the value too or just call the handler ? *)
        with
            ExtXml.Error e ->  lprintf out "%s = pers_settings.s_%d;\n" v !idx
      end;
      incr idx)
    pers_settings;
  left();
  lprintf out "}\n"
(*  end *)


(*
 Check if target t is marked as supported in the targets string.
 The targets string is a pipe delimited list of supported targets, e.g. "ap|nps"
 To specifiy a list with unsupported targets, prefix with !
 e.g. "!sim|nps" to mark support for all targets except sim and nps.
*)
let supports_target = fun t targets ->
  if String.length targets > 0 && targets.[0] = '!' then
    not (Str.string_match (Str.regexp (".*"^t^".*")) targets 0)
  else
    Str.string_match (Str.regexp (".*"^t^".*")) targets 0

let join_xml_files = fun xml_sys_files xml_user_files ->
  let dl_settings = ref []
  and rc_settings = ref [] in
  let target = try Sys.getenv "TARGET" with _ -> "" in
  List.iter (fun xml_file ->
    (* look for a specific name after settings file (in case of modules) *)
    let split = Str.split (Str.regexp "~") xml_file in
    let xml_file, name = match split with
    | [f; n] -> f, n
    | _ -> xml_file, ""
    in
    let xml = ExtXml.parse_file xml_file in
    let these_rc_settings =
      try Xml.children (ExtXml.child xml "rc_settings") with
          Not_found -> [] in
    let these_dl_settings =
      try
        (* test if the file is plain settings file or a module file *)
        let xml =
          if Xml.tag xml = "module"
          then begin
            (* test if the module is loaded or not *)
            if List.exists (fun n ->
              if Xml.tag n = "makefile" then begin
                let t = ExtXml.attrib_or_default n "target" Env.default_module_targets in
                supports_target target t
              end
              else false
              ) (Xml.children xml)
            then
              List.filter (fun t ->
                (* filter xml nodes and keep them if:
                 * it is a settings node
                 * the current target is supported in the 'target' attribute
                 * if no 'target' attribute always keep it
                 *)
                Xml.tag t = "settings" && supports_target target (ExtXml.attrib_or_default t "target" target)
              ) (Xml.children xml)
            else []
          end
          else begin
            (* if the top <settings> node has a target attribute,
               only add if matches current target *)
            let t = ExtXml.attrib_or_default xml "target" "" in
            if t = "" || (supports_target target t) then
              [xml]
            else
              []
          end
        in
        (* include settings if name is matching *)
        List.fold_left (fun l x ->
          if (ExtXml.attrib_or_default x "name" "") = name then
            l @ (Xml.children (ExtXml.child x "dl_settings"))
          else l
        ) [] xml
      with
      | Not_found -> [] in
    rc_settings := these_rc_settings @ !rc_settings;
    dl_settings := these_dl_settings @ !dl_settings)
    xml_user_files;

    (* add system settings grouped under the same tab *)
    let dl_sys = List.map (fun xml_file ->
      let xml = ExtXml.parse_file xml_file in
      (* take "second stage" dl_settings nodes *)
      try
        let dl = ExtXml.child xml "dl_settings" in
        Xml.children dl
      with Not_found -> []
    ) xml_sys_files in
    dl_settings := Xml.Element("dl_settings", [("name", "System")], List.rev (List.flatten dl_sys)) :: !dl_settings;

    (* return final node *)
  Xml.Element("rc_settings",[],!rc_settings), Xml.Element("dl_settings",[],!dl_settings)


let h_name = "SETTINGS_H"

let generate = fun settings xml_files out_xml out_file ->
  let out = open_out out_file in

  (*let rc_settings, dl_settings = join_xml_files xml_sys_files xml_user_files in*)

  (* generate XML concatenated file *)
  let settings_xml = List.fold_left (fun l s ->
    if List.length s.Settings.dl_settings > 0 then s.Settings.xml :: l else l
  ) [] settings
  in
  let settings_xml = List.rev settings_xml in (* list in correct order *)
  let dl_settings = Xml.Element("dl_settings", [], settings_xml) in
  let xml = Xml.Element ("settings", [], [dl_settings]) in
  let f = open_out out_xml in
  fprintf f "%s\n" (ExtXml.to_string_fmt xml);
  close_out f;

  (* generate C file *)
  begin_out out (String.concat " " xml_files) h_name;
  print_dl_settings out settings dl_settings;
  print_persistent_settings out settings dl_settings;
  finish_out out h_name

(*
let _ =
  if Array.length Sys.argv < 4 then
    failwith (Printf.sprintf "Usage: %s output_xml_file input_xml_file(s) input_xml_modules" Sys.argv.(0));
  and xml_files = Array.to_list (Array.sub Sys.argv 2 (Array.length Sys.argv - 2)) in
  (* split system settings and user settings based on '*' separator *)
  let xml_sys_files, xml_user_files, _ = List.fold_left (fun (sys, user, delim) x ->
    if x = "--" then (sys, user, true)
    else if delim then (sys, x :: user, delim)
    else (x :: sys, user, delim)) ([], [], false) xml_files
  in

  try
    printf "/* This file has been generated by gen_settings from %s */\n" (String.concat " " xml_files);
    printf "/* Version %s */\n" (Env.get_paparazzi_version ());
    printf "/* Please DO NOT EDIT */\n\n";

    printf "#ifndef %s\n" h_name;
    define h_name "";
    nl ();

    let rc_settings, dl_settings = join_xml_files xml_sys_files xml_user_files in

    let xml = Xml.Element ("settings", [], [rc_settings; dl_settings]) in
    let f = open_out Sys.argv.(1) in
    fprintf f "%s\n" (ExtXml.to_string_fmt xml);
    close_out f;

    lprintf "#define RCSettings(mode_changed) { \\\n";
    right ();
    parse_rc_modes rc_settings;
    left (); lprintf "}\n";

    print_dl_settings dl_settings;

    print_persistent_settings dl_settings;

    finish h_name
  with
      Xml.Error e -> prerr_endline (Xml.error e); exit 1
    | Dtd.Prove_error e ->  prerr_endline (Dtd.prove_error e); exit 1
    | Dtd.Parse_error e ->  prerr_endline (Dtd.parse_error e); exit 1
*)

