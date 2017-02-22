
"""Convert json data restructured text"""

import json
import os
import subprocess
try:
    from urllib.parse import urlparse
except ImportError:
    from urlparse import urlparse

from docutils import nodes, statemachine
from docutils.parsers.rst import Directive
from docutils.parsers.rst import directives
from docutils.statemachine import ViewList
from sphinx.util.nodes import nested_parse_with_titles
from sphinx.ext.autodoc import AutodocReporter


__version__ = '1.0'

def env_before_read_docs(app, env, docnames):
    # FIXME: update json/*.py so that we don't have to invoke as subprocess
    subprocess.call(['python', 'json/port_defs_to_json.py', env.srcdir,
                     app.outdir + '/ports.json'] + env.config.ev3dev_json_port_src_files)
    subprocess.call(['python', 'json/sensor_defs_to_json.py', '--source-dir', env.srcdir,
                     '--out-file', app.outdir + '/sensors.json', '--header-files']
                    + env.config.ev3dev_json_sensor_h_files + ['--source-files']
                    + env.config.ev3dev_json_sensor_src_files)
    subprocess.call(['python', 'json/motor_defs_to_json.py', '--source-dir', env.srcdir,
                     '--out-file', app.outdir + '/motors.json', '--header-files']
                    + env.config.ev3dev_json_motor_h_files + ['--source-files']
                    + env.config.ev3dev_json_motor_src_files)

def setup(app):
    u"""Setup function to hook into sphinx"""
    app.add_config_value('ev3dev_json_port_src_files', None, 'env')
    app.add_config_value('ev3dev_json_sensor_h_files', None, 'env')
    app.add_config_value('ev3dev_json_sensor_src_files', None, 'env')
    app.add_config_value('ev3dev_json_motor_h_files', None, 'env')
    app.add_config_value('ev3dev_json_motor_src_files', None, 'env')
    app.connect('env-before-read-docs', env_before_read_docs)
    app.add_directive('lego-sensor-list', LegoSensorList)
    app.add_directive('lego-sensor-data', LegoSensorData)
    app.add_directive('lego-motor-list', LegoMotorList)
    app.add_directive('lego-motor-data', LegoMotorData)
    app.add_directive('lego-port', LegoPort)
    app.add_directive('lego-port-list', LegoPortList)
    return dict(
        version=__version__,
        parallel_read_safe=True,
        parallel_write_safe=True
    )


class _Ev3devDirective(Directive):
    u"""Base class for this modules directives"""

    def _read(self, name):
        """Reads an object from a json file"""
        env = self.state.document.settings.env
        filename = os.path.join(env.app.outdir, name + '.json')
        env.note_dependency(os.path.abspath(filename))

        with open(filename, 'r') as f:
            return json.loads(f.read())

    def _table(self, colwidths, head, rows):
        table = nodes.table()

        tgroup = nodes.tgroup(cols=len(colwidths))
        table += tgroup

        for colwidth in colwidths:
            cspec = nodes.colspec(colwidth=colwidth)
            tgroup += cspec

        if head:
            thead = nodes.thead()
            tgroup += thead

            thead += head

        tbody = nodes.tbody()
        tgroup += tbody

        for row in rows:
            tbody += row

        return table

    def _row(self, items):
        row = nodes.row()

        for item in items:
            entry = nodes.entry()
            # make sure item is a list
            if isinstance(item, str):
                item = [item]
            result = ViewList(item)
            self.state.nested_parse(result, 0, entry)
            row += entry

        return row

    def _notes(self, device):
        """Extract and combine notes from a device

        Returns a section, including a title or ``None`` if there are no notes.
        """
        section = nodes.section(ids=[device['name'] + '-notes'],
                                names=[device['name'] + '\\ notes'])
        section += nodes.title(text='Notes')
        result = ViewList()
        has_notes = False
        if 'notes' in device:
            has_notes = True
            for line in device['notes']:
                result.append(line, device['source_file'], device['source_line'])
        if 'mode_info' in device:
            for mode in device['mode_info']:
                if 'notes' in mode:
                    has_notes = True
                    for line in mode['notes']:
                        result.append(line, device['source_file'], device['source_line'])
        if 'cmd_info' in device:
            for cmd in device['cmd_info']:
                if 'notes' in cmd:
                    has_notes = True
                    for line in cmd['notes']:
                        result.append(line, device['source_file'], device['source_line'])

        self.state.nested_parse(result, 0, section)

        return has_notes and section or None

    def _link(self, url, caption):
        """Create a ReST formatted link"""
        if not url:
            return caption
        return '`{} <{}>`_'.format(caption, url)

    def _pretty_sensor_type(self, name):
        if name == 'ev3-uart-sensor':
            return 'EV3/UART'
        if name == 'ev3-analog-sensor':
            return 'EV3/Analog'
        if name == 'nxt-i2c-sensor':
            return 'NXT/I2C'
        if name == 'nxt-analog-sensor':
            return 'NXT/Analog'
        if name == 'wedo-sensor':
            return 'Wedo/Analog'
        if name == 'other-i2c-sensor':
            return 'Other/I2C'
        if name == 'wedo-hub-sensor':
            return 'USB'
        # TODO: sphinx warning here
        return 'Unknown'


class LegoSensorList(_Ev3devDirective):
    u"""lego-sensor-list directive"""

    def run(self):
        """Hook for running this directive"""
        sensors = self._read('sensors')

        header = self._row(['Part Number', 'Name', 'Driver', 'Type', 'Auto-detect'])
        rows = self._generate_rows(sensors)

        table = self._table([1, 4, 1, 1, 1], header, rows)

        return [table]

    def _generate_rows(self, sensors):

        vendor_name = None
        for s in sensors:
            # only generate one row for each vendor name that serves as a group heading
            if 'vendor_name' in s and vendor_name != s['vendor_name']:
                vendor_name = s['vendor_name']

                strong = nodes.strong(text=vendor_name)
                para = nodes.paragraph()
                para += strong
                entry = nodes.entry(morecols=5)
                entry += para
                row = nodes.row()
                row += entry
                yield row

            # then generate one row for each sensor
            yield self._row([
                self._link(s.get('vendor_website', None), s.get('vendor_part_number', '')),
                ':ref:`{} <{}>`'.format(s.get('vendor_part_name', s.get('vendor_part_number', '')),
                                         s['url_name']),
                '``{}``'.format(s['name']),
                self._pretty_sensor_type(s['sensor_type']),
                self._auto_detect(s['name'], s['sensor_type']),
            ])

    def _auto_detect(self, name, typ):
        if typ == 'ev3-uart-sensor':
            return 'Y'
        if typ == 'ev3-analog-sensor':
            return 'Y'
        if typ == 'nxt-analog-sensor':
            if name == 'lego-nxt-touch':
                return 'Y [#detect-lego-nxt-touch]_'
            if name in ('nxt-analog', 'lego-nxt-light'):
                return 'Y'
            if name == 'di-dflex':
                return 'N [#detect-di-dflex]_'
            return 'N [#detect-nxt-analog]_'
        if typ == 'nxt-i2c-sensor':
            if name == 'mi-xg1300l':
                return 'N [#detect-mi-xg1300l]_'
            return 'Y'
        if typ == 'other-i2c-sensor':
            return 'N [#detect-other-i2c]_'
        if typ == 'wedo-sensor':
            return 'Y'
        if typ == 'wedo-hub-sensor':
            return 'Y'
        return 'N'


class LegoSensorData(_Ev3devDirective):
    u"""lego-sensor-data directive"""

    def run(self):
        """Hook for running this directive"""
        sensors = self._read('sensors')

        return [s for s in self._sections(sensors)]

    def _sections(self, sensors):
        """Generate a section for each sensor"""

        for s in sensors:
            dummy = nodes.section()
            result = ViewList()
            result.append('.. _{}:'.format(s['name']),
                          source=s['source_file'],
                          offset=s['source_line'])
            if s['name'] != s['url_name']:
                result.append('.. _{}:'.format(s['url_name']),
                              source=s['source_file'],
                              offset=s['source_line'])
            self.state.nested_parse(result, 0, dummy)
            for c in dummy.children:
                yield c

            # FIXME: not sure why this does not have the same effect as above
            # target = nodes.target(ids=[s['url_name']], names=[s['url_name']])
            # yield target

            section = nodes.section(ids=[s['name']], names=[s['name']])

            title_text = s.get('vendor_part_name', None) or s['vendor_part_number']
            if 'vendor_name' in s:
                title_text = '{} {}'.format(s['vendor_name'], title_text)
            title = nodes.title(text=title_text)
            section += title

            info_section = nodes.section(ids=[s['name'] + '-info'],
                                         names=[s['name'] + '\\ info'])
            info_title = nodes.title(text='General Info')
            info_section += info_title
            info_table = self._table([1, 1], None, [
                r for r in self._info_rows(s)
            ])
            info_section += info_table
            section += info_section

            if 'mode_info' in s:
                modes_section = nodes.section(ids=[s['name'] + '-modes'],
                                              names=[s['name'] + '\\ modes'])
                modes_title = nodes.title(text='Modes')
                modes_section += modes_title
                modes_header = self._row([
                    'Mode',
                    'Description',
                    'Units',
                    'Decimals',
                    'Num. Values',
                    'Values'
                ])
                modes_rows = [r for r in self._modes_rows(s)]
                modes_table = self._table([3, 6, 3, 1, 1, 6], modes_header, modes_rows)
                modes_section += modes_table
                section += modes_section

                cmds_section = nodes.section(ids=[s['name'] + '-commands'],
                                             names=[s['name'] + '\\ commands'])
                cmds_title = nodes.title(text='Commands')
                cmds_section += cmds_title
                if 'cmd_info' in s:
                    cmds_table = self._table([1, 6], self._row(['Command', 'Description']),
                                             [r for r in self._cmds_rows(s)])
                    cmds_section += cmds_table
                else:
                    cmds_paragraph = nodes.paragraph(
                        text='This sensor does not support commands.')
                    cmds_section += cmds_paragraph
                section += cmds_section

            notes = self._notes(s)
            if notes:
                section += notes

            yield section

    def _info_rows(self, sensor):
        yield self._row(['Driver Name', '``{}``'.format(sensor['name'])])

        if 'vendor_website' in sensor:
            hostname = urlparse(sensor['vendor_website']).hostname
            # have to use an anonymous link since label (hostname in this case)
            # can be the same for multiple sensors
            link = ['`{}`__'.format(hostname), '', '__ {}'.format(sensor['vendor_website'])]
            yield self._row(['Website', link])

        yield self._row(['Connection Type', self._pretty_sensor_type(sensor['sensor_type'])])

        if 'default_address' in sensor:
            default_address = sensor['default_address']
            if 'default_address_footnote' in sensor:
                default_address += ' ' + sensor['default_address_footnote']
            yield self._row(['Default Address', default_address])

        if 'vendor_id' in sensor:
            vid = '``{}``'.format(sensor['vendor_id'])
            if 'alt_vendor_id' in sensor:
                vid += ' (or ``{}``)'.format(sensor['alt_vendor_id'])
            if 'vendor_id_footnote' in sensor:
                vid += ' ' + sensor['vendor_id_footnote']
            yield self._row(['Vendor ID', vid])

            pid = '``{}``'.format(sensor['product_id'])
            if 'product_id_footnote' in sensor:
                pid += ' ' + sensor['product_id_footnote']
            yield self._row(['Product ID', pid])

        if 'num_modes' in sensor:
            yield self._row(['Number of Modes', sensor['num_modes']])

    def _modes_rows(self, sensor):
        for m in sensor['mode_info']:
            mode_name = '``{}``'.format(m['name'])
            if 'name_footnote' in m:
                mode_name += ' ' + m['name_footnote']

            if 'units' in m:
                units = '``{}``'.format(m['units'])
                if 'units_description' in m:
                    units += ' ({})'.format(m['units_description'])
            else:
                units = '*none*'
            if 'units_footnote' in m:
                units += ' ' + m['units_footnote']

            values = []
            for i in range(0, int(m.get('data_sets', 1))):
                value = 'value{}'.format(i)
                value_text = '``{}``: '.format(value)
                if value in m:
                    value_text += m[value]
                if value + '_footnote' in m:
                    value_text += ' ' + m[value + '_footnote']
                values.append(value_text)
                values.append('')

            yield self._row([
                mode_name,
                m['description'],
                units,
                m.get('decimals', '0'),
                m.get('data_sets', '1'),
                values,
            ])

    def _cmds_rows(self, sensor):
        for c in sensor['cmd_info']:
            cmd_name = '``{}``'.format(c['name'])
            if 'name_footnote' in c:
                cmd_name += ' ' + c['name_footnote']
            yield self._row([cmd_name, c['description']])


class LegoMotorList(_Ev3devDirective):
    u"""lego-motor-list directive"""

    def run(self):
        """Hook for running this directive"""
        motors = self._read('motors')

        header = self._row(['Part Number', 'Name', 'Driver', 'Type', 'Auto-detect'])
        rows = self._generate_rows(motors)

        table = self._table([1, 4, 1, 1, 1], header, rows)

        return [table]

    def _generate_rows(self, motors):
        vendor_name = None
        for m in motors:
            # only generate one row for each vendor name that serves as a group heading
            if 'vendor_name' in m and vendor_name != m['vendor_name']:
                vendor_name = m['vendor_name']

                strong = nodes.strong(text=vendor_name)
                para = nodes.paragraph()
                para += strong
                entry = nodes.entry(morecols=5)
                entry += para
                row = nodes.row()
                row += entry
                yield row

            # then generate one row for each sensor
            yield self._row([
                self._link(m.get('vendor_website', None), m['vendor_part_number']),
                m['vendor_part_name'],
                '``{}``'.format(m['name']),
                m['motor_type'],
                self._auto_detect(m['name'], m['motor_type']),
            ])

    def _auto_detect(self, name, typ):
        if typ == 'ev3':
            if name in ('lego-ev3-l-motor', 'lego-ev3-m-motor', 'lego-nxt-motor'):
                return 'Y [#motor-autodetect]_'
            return 'N [#motor-autodetect]_'
        return 'N'


class LegoMotorData(_Ev3devDirective):
    u"""lego-motor-data directive"""

    def run(self):
        """Hook for running this directive"""
        motors = self._read('motors')

        return [s for s in self._sections(motors)]

    def _sections(self, motors):
        """Generate a section for each sensor"""

        for m in motors:
            dummy = nodes.section()
            result = ViewList()
            result.append('.. _{}:'.format(m['name']),
                          source=m['source_file'],
                          offset=m['source_line'])
            if m['name'] != m['url_name']:
                result.append('.. _{}:'.format(m['url_name']),
                              source=m['source_file'],
                              offset=m['source_line'])
            self.state.nested_parse(result, 0, dummy)
            for c in dummy.children:
                yield c

            # FIXME: not sure why this does not have the same effect as above
            # target = nodes.target(ids=[s['url_name']], names=[s['url_name']])
            # yield target

            section = nodes.section(ids=[m['name']], names=[m['name']])

            title_text = m.get('vendor_part_name', None) or m['vendor_part_number']
            if 'vendor_name' in m:
                title_text = '{} {}'.format(m['vendor_name'], title_text)
            title = nodes.title(text=title_text)
            section += title

            info_section = nodes.section(ids=[m['name'] + '-info'],
                                         names=[m['name'] + '\\ info'])
            info_title = nodes.title(text='General Info')
            info_section += info_title
            info_table = self._table([1, 1], None, [
                r for r in self._info_rows(m)
            ])
            info_section += info_table
            section += info_section

            notes = self._notes(m)
            if notes:
                section += notes

            yield section

    def _info_rows(self, motor):
        yield self._row(['Driver Name', '``{}``'.format(motor['name'])])

        if 'vendor_website' in motor:
            hostname = urlparse(motor['vendor_website']).hostname
            # have to use an anonymous link since label (hostname in this case)
            # can be the same for multiple sensors
            link = ['`{}`__'.format(hostname), '', '__ {}'.format(motor['vendor_website'])]
            yield self._row(['Website', link])

        yield self._row(['Connection Type', motor['motor_type']])


class LegoPortList(_Ev3devDirective):
    u"""lego-port-list directive"""

    def run(self):
        """Hook for running this directive"""
        ports = self._read('ports')

        head = self._row(['Name', 'Description', 'Connection Types', 'Module'])
        rows = [self._row([
            '``{}``'.format(ports[p]['name']),
            ':ref:`{} <{}>`'.format(ports[p]['description'], ports[p]['struct_name']),
            ports[p]['connection_types'],
            '``{}``'.format(ports[p]['module'])
        ]) for p in ports]

        table = self._table([1, 3, 3, 1], head, rows)
        return [table]


class LegoPort(_Ev3devDirective):
    u"""lego-port directive for creating docs for a single type of port from ports.json"""
    required_arguments = 1

    def run(self):
        """Hook for running directive"""
        ports = self._read('ports')
        port = ports[self.arguments[0]]
        info_table = self._info_table(port)
        modes_table = self._modes_table(port)
        notes = self._notes(port)

        new_nodes = [info_table, modes_table]
        if notes:
            new_nodes.append(notes)

        return new_nodes

    def _info_table(self, port):
        section = nodes.section(ids=[port['name'] + '-general-info'],
                                names=[port['name'] + '\\ general\\ info'])
        section += nodes.title(text='General Info')

        prefix = '``{}``'.format(port['prefix'])
        if 'prefix_footnote' in port:
            prefix += ' ' + port['prefix_footnote']

        table = self._table([1, 2], None, [
            self._row(['Module', '``{}``'.format(port['module'])]),
            self._row(['Driver name', '``{}``'.format(port['name'])]),
            self._row(['Connection types', port['connection_types']]),
            self._row(['Connection prefix', prefix]),
            self._row(['Number of modes', str(port['num_modes'])]),
        ])
        section += table

        return section

    def _modes_table(self, port):
        section = nodes.section(ids=[port['name'] + '-modes'],
                                names=[port['name'] + '\\ modes'])
        section += nodes.title(text='Modes')

        table = self._table([1, 2], self._row(['Name', 'Description']),
                            [self._modes_table_row(m) for m in port['mode_info']])
        section += table

        return section

    def _modes_table_row(self, mode):
        name = '``{}``'.format(mode['name'])
        if 'name_footnote' in mode:
            name += ' ' + mode['name_footnote']

        return self._row([name, mode['description']])
