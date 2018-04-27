#!/usr/bin/ruby

require 'optparse'
require 'fileutils'

include FileUtils

rels = {
  firmware: "../../../Firmware",
  sitl_gazebo: "../sitl_gazebo",

  workspace_firmware: "fw"
}

#script dir
abs = {
  home: __dir__
}

#options
opts = {
  n: 1,
  r: 10000,
  f: "ekf2",
  o: {},
  gazebo_model: "iris",
  base_port: 15010,
  port_step: 10,
  distance: 2,
  build_label: "default",

  workspace: "workspace",
  gazebo: "gazebo",
  catkin_ws: "workspace/catkin_ws",
}

#sitl_gazebo
model_opts_open = "<?xml version=\"1.0\" ?>
<options>\n"
model_opts_close = '</options>'


def xspawn(term_name, cmd, debug)
  term = debug ? "xterm -T #{term_name} -hold -e" : ""
  pid = spawn("#{term} #{cmd}", [:out, :err]=>"/dev/null")
  Process.detach(pid)
end

def create_fcu_files(opts, m_num, mix_file, rel_mix, init_str)
  rc_file="rcS#{m_num}"

  unless File.exist?(rc_file)
    mkdir_p "rootfs/fs/microsd"
    mkdir_p "rootfs/eeprom"
    touch "rootfs/eeprom/parameters"

    cp mix_file, "./"

    #generate rc file
    rc = init_str.sub('param set MAV_TYPE',"param set MAV_SYS_ID #{m_num}\nparam set MAV_TYPE")
    rc.sub!(File.dirname(rel_mix) + '/','')
    unless opts[:logging]
      rc.sub!(/sdlog2 start.*\n/,'')
      rc.sub!(/logger start.*\n/,'')
    end
    rc.sub!(/.*OPTICAL_FLOW_RAD.*\n/,'') unless opts[:optical_flow]

    rc.sub!(/(simulator start.*)/, "\\1 -u #{@sim_port}")

    rc.gsub!(/(mavlink start .*-r) \S+/,"\\1 #{opts[:r]}")

    rc.sub!(/(mavlink start .*-u) 14556/, "\\1 #{@mav_port} -o #{@mav_oport}")
    rc.gsub!("-u 14556","-u #{@mav_port}")

    rc.sub!("-u 14557","-u #{@mav_port2}")
    rc.sub!("-r 250 -s HIGHRES_IMU -u #{@mav_port}", "-r #{opts[:imu_rate]} -s HIGHRES_IMU -u #{@mav_port2}") if opts[:imu_rate]
    rc.sub!("-o 14540","-o #{@mav_oport2}")
    rc.sub!("gpssim start","param set MAV_USEHILGPS 1") if opts[:hil_gps]

    File.open(rc_file, 'w') { |out| out << rc }
  end

  rc_file
end

def check_expanded_path(file_name, dir = nil, msg = nil)
  p = File.expand_path(file_name, dir)

  unless File.exist?(p)
    puts "#{p} does not exist" + (msg ? ", #{msg}" : "")
    exit
  end

  p
end

def xml_elements(kv)
  s = ""
  kv.each { |k, v| s<<"<#{k}>#{v}</#{k}>" }

  s
end

#options
op = OptionParser.new do |op|
  op.banner = "Usage: #{__FILE__} [options] [world_file]"

  op.on("-n NUM", Integer, "number of instances") { |p| opts[:n] = p }
  op.on("-r RATE", Integer, "px4 data rate") { |p| opts[:r] = p }
  op.on("-f FILTER", "px4 filter") { |p| opts[:f] = p }
  op.on("-i NAME", "px4 model init file") { |p| opts[:i] = p }
  op.on("-o PARAM=VALUE", /(.+)=(.+)/, "gazebo model option") { |p, k, v| opts[:o][k] = v }

  op.on("--gazebo_model NAME", "gazebo model name") { |p| opts[:gazebo_model] = p }
  op.on("--imu_rate IMU_RATE", Integer, "imu rate") { |p| opts[:imu_rate] = p }
  op.on("--hil_gps", "turn on hil_gps mode") { opts[:hil_gps] = true }
  op.on("--plugin_lists PATH", "path to mavros pluginlists.yaml") { |p| opts[:plugin_lists] = p }
  op.on("--logging", "turn on logging") { opts[:logging] = true }
  op.on("--debug", "debug mode") { opts[:debug] = true }
  op.on("--nomavros", "without mavros") { opts[:nomavros] = true }
  op.on("--workspace PATH", "path to workspace") { |p| opts[:workspace] = p }
  op.on("--gazebo PATH", "path to gazebo resources") { |p| opts[:gazebo] = p }
  op.on("--catkin_ws PATH", "path to catkin workspace") { |p| opts[:catkin_ws] = p }
  op.on("--base_port PORT", Integer, "base port") { |p| opts[:base_port] = p }
  op.on("--port_step STEP", Integer, "port step") { |p| opts[:port_step] = p }
  op.on("--distance DISTANCE", Integer, "distance between models") { |p| opts[:distance] = p }
  op.on("--optical_flow", "turn on optical flow") { opts[:optical_flow] = true }
  op.on("--firmware PATH", "path to firmware folder") { |p| opts[:firmware] = p }
  op.on("--sitl_gazebo PATH", "path to sitl_gazebo folder") { |p| opts[:sitl_gazebo] = p }
  op.on("--build_label NAME", "build label") { |p| opts[:build_label] = p }
  op.on("--legacy", "legacy logic") { |p| opts[:legacy] = true }

  op.on("--restart", "soft restart") do
    opts[:restart] = true
    puts "restarting ..."
  end

  op.on("-h", "help and show defaults") do
    puts op
    puts "\nDefault options: #{opts}"

    opts[:help] = true
  end
end
op.parse!
opts[:gazebo_world] = ARGV[0]

target = "posix_sitl_#{opts[:build_label]}"
fw_dir = opts[:legacy] ? "build_#{target}/src/firmware/posix" : "build/#{target}"

#update rels
rels.update({
  firmware_bin: "#{fw_dir}/px4",
  firmware_init: "posix-configs/SITL/init/#{opts[:f]}/#{opts[:i] || opts[:gazebo_model]}",
  gazebo_world: "worlds/#{opts[:gazebo_model]}.world"
})
msgs = {
  firmware_init: "use valid -f, -i, --gazebo_model options",
  gazebo_world: "invalid --gazebo_model option"
}

#do not check
for sym in [:workspace, :gazebo, :catkin_ws]
  abs[sym] = File.expand_path(opts[sym])
end

#check
for sym in [:firmware, :sitl_gazebo, :plugin_lists, :gazebo_world]
  abs[sym] = check_expanded_path(opts[sym]) if opts[sym]
end

#set defaults if not set
for sym in [:firmware, :sitl_gazebo]
  abs[sym] = check_expanded_path(rels[sym], abs[:home]) unless abs[sym]
end

#check firmware paths
for sym in [:firmware_bin, :firmware_init]
  abs[sym] = check_expanded_path(rels[sym], abs[:firmware], msgs[sym])
end

#get gazebo world file
unless abs[:gazebo_world]
  a = []
  for sym in [:gazebo, :sitl_gazebo]
    a << abs[sym]
    ap = File.expand_path(rels[:gazebo_world], abs[sym])
    if File.exist?(ap)
      abs[:gazebo_world] = ap
      break
    end
  end

  unless abs[:gazebo_world]
    puts msgs[:gazebo_world] + ", " +rels[:gazebo_world] + " does not exist in " + a.join(", ")
    exit
  end
end

#contents
contents = {}
for sym in [:firmware_init, :gazebo_world]
  contents[sym] = File.read(abs[sym])
end
world_name = contents[:gazebo_world][/<world name="(.*)">/, 1]
contents[:gazebo_world].sub!(/\n[^<]*<include>[^<]*<uri>model:\/\/iris[^<]*<\/uri>.*?<\/include>/m, "")

#update with contents
rels.update({
  firmware_mix: contents[:firmware_init][/mixer load .*/].split.last,

  workspace_world: world_name + ".world",
  workspace_opts: world_name + ".xml"
})

abs[:firmware_mix] = check_expanded_path(rels[:firmware_mix], abs[:firmware], "invalid mixer in #{abs[:firmware_init]}")

for sym in [:workspace_firmware, :workspace_world, :workspace_opts]
  abs[sym] = File.expand_path(rels[sym], abs[:workspace])
end

if opts[:help]
  puts "\nDefault absolute paths: #{abs}"
  puts
  exit
end

#init
cd abs[:home]

if opts[:restart]
  system("./kill_px4.sh")
else
  system("./kill_sitl.sh")
end
sleep 1

#start

fw_name = rels[:firmware_bin].split('/').last
unless File.exist?(abs[:workspace_firmware] + '/' + fw_name)
  mkdir_p abs[:workspace_firmware]
  cp abs[:firmware_bin], abs[:workspace_firmware]
end

model_incs = ""
model_opts = ""
model_opts += "  <model>" + xml_elements(opts[:o]) + "</model>\n" unless opts[:o].empty?
opts[:n].times do |i|
  x=i*opts[:distance]
  m_index=i
  m_num=i+1

  @mav_port = opts[:base_port] + m_index*opts[:port_step]
  @mav_port2 = @mav_port + 1

  @mav_oport = @mav_port + 5
  @mav_oport2 = @mav_port + 6

  @sim_port = @mav_port + 9

  @gcs_inport = @mav_port + 2000

  model_name="#{opts[:gazebo_model]}#{m_num}"

  cd(abs[:workspace_firmware]) {
    mkdir_p model_name

    cd(model_name) {
      rc_file = create_fcu_files(opts, m_num, abs[:firmware_mix], rels[:firmware_mix], contents[:firmware_init])

      #run firmware
      xspawn("#{fw_name}-#{m_num}", "../#{fw_name} -d #{rc_file}", opts[:debug])
    }
  }

  #generate model
  n = "<name>#{model_name}</name>"

  model_incs += "    <include>#{n}<uri>model://#{opts[:gazebo_model]}</uri><pose>#{x} 0 0 0 0 0</pose></include>\n" unless contents[:gazebo_world].include?(n)
  model_opts += "  <model name=\"#{model_name}\">" + xml_elements(:mavlink_udp_port => @sim_port) + "</model>\n"

  cd("mavros") {
    sleep 1

    pl="plugin_lists:=#{abs[:plugin_lists]}" if abs[:plugin_lists]
    launch_opts = "fcu_url:=udp://127.0.0.1:#{@mav_oport2}@127.0.0.1:#{@mav_port2} gcs_inport:=#{@gcs_inport}"

    xspawn("mavros-#{m_num}", "./roslaunch.sh #{abs[:catkin_ws]} num:=#{m_num} #{pl} #{launch_opts}", opts[:debug])

  } unless opts[:restart] or opts[:nomavros]

end

if opts[:restart]
  system("gz world -o")
else
  File.open(abs[:workspace_opts], 'w') do |out|
    out << model_opts_open
    out << model_opts
    out << model_opts_close
  end

  #run gzserver
  File.open(abs[:workspace_world], 'w') do |out|
    contents[:gazebo_world].each_line {|l|
      out << l
      if l =~ /.*<world.*\n/
        out << model_incs
      end
    }

  end

  cd(abs[:workspace]) {
    xspawn("gazebo", abs[:home] + "/gazebo.sh #{rels[:workspace_world]} #{abs[:gazebo]} #{abs[:sitl_gazebo]}", opts[:debug])
  }
end
