clear;

% look for a bag from the track_status topic
trackstatus_file = dir('*_track_status.txt');
if (~isempty(trackstatus_file)) 
    [track.time, track.headseq, track.headstamp,track.headframe_id, track.t_track,track.t_corr,track.t_clones,track.fps,track.corr_offset(:,1),track.corr_offset(:,2),track.corr_offset(:,3),track.ptw(:,1),track.ptw(:,2),track.ptwt,track.zncc,track.RMS,track.ctrlstop,track.H(1,1,:),track.H(1,2,:),track.H(1,3,:),track.H(2,1,:),track.H(2,2,:),track.H(2,3,:),track.H(3,1,:),track.H(3,2,:),track.H(3,3,:) ] = textread(trackstatus_file.name, '','delimiter', ',','headerlines',1);
end

% look for a bag from the dvs_status topic
dvsstatus_file = dir('*_dvs_status.txt');
if (~isempty(dvsstatus_file)) 
%[dvs.time, track.headseq, track.headstamp,track.headframe_id, track.t_track,track.t_corr,track.t_clones,track.fps,track.corr_offset(:,1),track.corr_offset(:,2),track.corr_offset(:,3),track.ptw(:,1),track.ptw(:,2),track.ptwt,track.zncc,track.RMS,track.ctrlstop,track.H(1,1,:),track.H(1,2,:),track.H(1,3,:),track.H(2,1,:),track.H(2,2,:),track.H(2,3,:),track.H(3,1,:),track.H(3,2,:),track.H(3,3,:) ] = textread(dvsstatus_file.name, '','delimiter', ',','headerlines',1);
    [dvs.time,dvs.headseq,dvs.headstamp,dvs.headframe_id,dvs.e(:,1),dvs.e(:,2),dvs.e(:,3),dvs.e(:,4),dvs.e(:,5),dvs.e(:,6),dvs.v_c(:,1),dvs.v_c(:,2),dvs.v_c(:,3),dvs.v_c(:,4),dvs.v_c(:,5),dvs.v_c(:,6),dvs.v_r(:,1),dvs.v_r(:,2),dvs.v_r(:,3),dvs.v_r(:,4),dvs.v_r(:,5),dvs.v_r(:,6),dvs.v_r_mes(:,1),dvs.v_r_mes(:,2),dvs.v_r_mes(:,3),dvs.v_r_mes(:,4),dvs.v_r_mes(:,5),dvs.v_r_mes(:,6),dvs.pose(:,1),dvs.pose(:,2),dvs.pose(:,3),dvs.pose(:,4),dvs.pose(:,5),dvs.pose(:,6),dvs.fps ] = textread(dvsstatus_file.name, '','delimiter', ',','headerlines',1);
end

save('rawdata.mat');

