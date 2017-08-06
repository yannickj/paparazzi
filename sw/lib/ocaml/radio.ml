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
 * Radio control module for parsing XML config files
 *)

type channel = {
  name: string; (* a.k.a. function in DTD *)
  min: int;
  max: int;
  neutral: int;
  average: bool; (* average data *)
  reverse: bool; (* reverse min/max parameters *)
}

let parse_channel = function
  | Xml.Element ("channel", attribs, []) ->
      let iget = fun attrib -> int_of_string (List.assoc attrib attribs) in
      let bget = fun attrib -> try List.assoc attrib attribs <> "0" with Not_found -> false in
      {
        name = List.assoc "name" attribs;
        min = iget "min";
        max = iget "max";
        neutral = iget "netral";
        average = bget "average";
        reverse = bget "reverse";
      }
  | _ -> failwith "Radio.parse_channel: unreachable"


type pulse = PositivePulse | NegativePulse

type t = {
  name: string;
  data_min: int;
  data_max: int;
  sync_min: int;
  sync_max: int;
  pulse_type: pulse;
  channels: channel list;
  xml: Xml.xml;
}

let from_xml = function
  | Xml.Element ("radio", attribs, channels) as xml ->
      let get = fun attrib -> List.assoc attrib attribs in
      let iget = fun attrib -> int_of_string (List.assoc attrib attribs) in
      {
        name = get "name";
        data_min = iget "data_min";
        data_max = iget "data_max";
        sync_min = iget "sync_min";
        sync_max = iget "sync_max";
        pulse_type = begin match get "name" with
          | "POSITIVE" -> PositivePulse
          | "NEGATIVE" -> NegativePulse
          | _ -> failwith "Radio.from_xml: unknown pulse type"
        end;
        channels = List.map parse_channel channels;
        xml;
      }
  | _ -> failwith "Radio.from_xml: unreachable"

