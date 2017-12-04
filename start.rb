#!/usr/bin/ruby

require 'optparse'
require 'fileutils'

include FileUtils

def xspawn(term_name, cmd, debug)
  term = debug ? "xterm -T #{term_name} -hold -e" : ""
  pid = spawn("#{term} #{cmd}", [:out, :err]=>"/dev/null")
  Process.detach(pid)
end

base_port=15010
port_step=10
distance=2
sitl_base_path="px4dir"

wrk_dir = __dir__ + '/'

#options
all_model_names = ["iris", "iris_opt_flow"]
opts = { model: "iris", num: 1, rate: 10000, filter: "ekf2" }

op = OptionParser.new do |op|
  op.banner = "Usage: #{__FILE__} [options] [world_file]"

  op.on("-m MODEL", all_model_names, all_model_names.join(", ")) do |m|
    opts[:model] = m
  end

  op.on("--gazebo_model MODEL", "gazebo model name") do |p|
    opts[:gazebo_model] = p
  end

  op.on("-n NUM", Integer, "number of instances") do |n|
    opts[:num] = n
  end

  op.on("-r RATE", Integer, "px4 data rate") do |r|
    opts[:rate] = r
  end

  op.on("-g INTERVAL", Float, "gps update interval") do |g|
    opts[:gps_interval] = g
  end

  op.on("--imu_rate IMU_RATE", Integer, "imu rate") do |p|
    opts[:imu_rate] = p
  end

  op.on("--hil_gps", "turn on hil_gps mode") do
    opts[:hil_gps] = true
  end

  op.on("--plugin_lists PATH", "path to mavros pluginlists.yaml") do |p|
    opts[:plugin_lists] = p
  end

  op.on("-f FILTER", "filter") do |o|
    opts[:filter] = o
  end

  op.on("--logging", "turn on logging") do
    opts[:logging] = true
  end

  op.on("--hitl", "HITL mode") do
    opts[:hitl] = true
  end

  op.on("--restart", "soft restart") do
    opts[:restart] = true
  end

  op.on("--debug", "debug") do
    opts[:debug] = true
  end

  op.on("-h", "help") do
    puts op
    exit
  end
end
op.parse!

users_world_fname = ARGV[0]
if users_world_fname
  unless File.exist?(users_world_fname)
    puts op
    exit
  end

  unless opts[:model]
    str = File.read(users_world_fname)
    all_model_names.each { |m|
      uri = "<uri>model://#{m}</uri>"
      if str.include?(uri)
        opts[:model] = m
        opts[:num] = str.lines.grep(/.*#{uri}.*/).size

        break
      end
    }
  end
end

model = opts[:model]
gazebo_model = opts[:gazebo_model] || opts[:model]

#Firmware
px4_fname="px4"
px4_dir="Firmware/build_posix_sitl_default/src/firmware/posix/"
rc_script="Firmware/posix-configs/SITL/init/#{opts[:filter]}/#{model}"

#sitl_gazebo
#model_path = "sitl_gazebo/models/#{model}/#{model}.sdf"
world_path = "sitl_gazebo/worlds/#{model}.world"
world_fname="default.world"
model_incs = ""
models_opts_fname = "options.xml"
model_opts = ""
model_opts_open = "<?xml version=\"1.0\" ?>
<options>\n"
model_opts_close = '</options>'

#mavros
mavros_dir="mavros"

if opts[:restart]
  system("pkill px4")
else
  system("./kill_sitl.sh")
end
sleep 1

unless Dir.exist?(sitl_base_path)
  mkdir sitl_base_path
  cp px4_dir+px4_fname, sitl_base_path
end

opts[:num].times do |i|
  x=i*distance
  m_index=i
  m_num=i+1

  next if opts[:hitl] and opts[:num] != m_num

  mav_port = base_port + m_index*port_step
  mav_port2 = mav_port + 1

  mav_oport = mav_port + 5
  mav_oport2 = mav_port + 6

  hil_gps_port = mav_port + 8
  sim_port = mav_port + 9

  bridge_port = mav_port + 2000

  model_name="#{gazebo_model}#{m_num}"

  cd(sitl_base_path) {
    mkdir_p model_name

    cd(model_name) {
      rc_file="rcS#{m_num}"

      unless File.exist?(rc_file)
        mkdir_p "rootfs/fs/microsd"
        mkdir_p "rootfs/eeprom"
        touch "rootfs/eeprom/parameters"

        cp wrk_dir+"Firmware/ROMFS/px4fmu_common/mixers/quad_w.main.mix", "./"

        #generate rc file
        rc1 ||= File.read(wrk_dir + rc_script)
        rc = rc1.sub('param set MAV_TYPE',"param set MAV_SYS_ID #{m_num}\nparam set MAV_TYPE")
        rc.sub!('ROMFS/px4fmu_common/mixers/','')
        unless opts[:logging]
          rc.sub!(/sdlog2 start.*\n/,'')
          rc.sub!(/logger start.*\n/,'')
        end
        rc.sub!(/.*OPTICAL_FLOW_RAD.*\n/,'') if model=="iris"

        rc.sub!(/simulator start -s.*$/,"simulator start -s -u #{sim_port}")

        rc.gsub!("-r 4000000","-r #{opts[:rate]}")

        rc.gsub!("-u 14556","-u #{mav_port}")
        rc.sub!("mavlink start -u #{mav_port}","mavlink start -u #{mav_port} -o #{mav_oport}")

        rc.sub!("-u 14557","-u #{mav_port2}")
        rc.sub!("-r 250 -s HIGHRES_IMU -u #{mav_port}", "-r #{opts[:imu_rate]} -s HIGHRES_IMU -u #{mav_port2}") if opts[:imu_rate]
        rc.sub!("-o 14540","-o #{mav_oport2}")
        rc.sub!("gpssim start","param set MAV_USEHILGPS 1") if opts[:hil_gps]

        File.open(rc_file, 'w') { |out| out << rc }
      end

      #run px4
      xspawn("px4-#{m_num}", "../#{px4_fname} -d #{rc_file}", opts[:debug])
    }
  } unless opts[:hitl]

  #generate model
  model_incs += "    <include>
      <uri>model://#{gazebo_model}</uri>
      <pose>#{x} 0 0 0 0 0</pose>
      <name>#{model_name}</name>
    </include>\n"

  model_opts += "  <model>
    <name>#{model_name}</name>
    <mavlink_udp_port>#{sim_port}</mavlink_udp_port>\n"
  model_opts += "    <gps_update_interval>#{opts[:gps_interval]}</gps_update_interval>\n"  if opts[:gps_interval]
  model_opts += "    <imu_rate>#{opts[:imu_rate]}</imu_rate>\n"  if opts[:imu_rate]
  model_opts += "    <hil_gps_port>#{hil_gps_port}</hil_gps_port>\n" if opts[:hil_gps]
  model_opts += "  </model>\n"

  cd(mavros_dir) {
    sleep 1

    pl="plugin_lists:=#{File.expand_path(opts[:plugin_lists], wrk_dir)}" if opts[:plugin_lists]
    launch_opts = opts[:hitl] ? "bridge_on:=true bridge_inport:=#{bridge_port} fcu_url:=/dev/ttyACM0:921600 gcs_inport:=#{sim_port}" : "fcu_url:=udp://127.0.0.1:#{mav_oport2}@127.0.0.1:#{mav_port2} gcs_inport:=#{bridge_port}"

    xspawn("mavros-#{m_num}", "roslaunch px4_num.launch num:=#{m_num} #{pl} #{launch_opts}", opts[:debug])

  } unless opts[:restart]
end

if opts[:restart]
  system("gz world -o")
else
  File.open(models_opts_fname, 'w') do |out|
    out << model_opts_open
    out << model_opts
    out << model_opts_close
  end

  #run gzserver
  if users_world_fname
    cp users_world_fname, world_fname
  else
    world_sdf = File.read(world_path)
    File.open(world_fname, 'w') do |out|
      out << world_sdf.sub!(/.*<include>.*\n.*<uri>model:\/\/iris.*<\/uri>.*\n.*<\/include>.*\n/, model_incs)
    end
  end

  xspawn("gazebo", "./gazebo.sh #{world_fname}", opts[:debug])
end
