%% run_3pi_serial_scan.m
% Command-triggered data collection for Pololu 3pi+ 32U4 scans.
% Arduino should output lines:
%   - Deadzone: DZ,wheel,pwm,repeat,cps,is_moving
%   - RatioScan: DATA,dir,pwm,repeat,cpsL,cpsR,ratio
% And finish with:
%   # done
%
% This script:
%   - Creates output folder: 3pi_results/<Kind>/<YYYY-MM-DD>/<HHMMSS>/
%   - Sends command (DZ or RS) immediately (no READY dependency)
%   - Prints progress in MATLAB Command Window while receiving
%   - Saves raw lines + CSV + summary CSV + FIG plots

%% ====== USER SETTINGS ======
port = "COM4";          % e.g. "COM6"
baud = 115200;
cmd  = "DZ";            % "DZ" or "RS"
timeout_s = 600;        % max wait time in seconds

baseOutDir = fullfile(pwd, "3pi_results");

% Console printing behavior
print_all_lines = false;   % false = progress only (recommended), true = print every line (spammy)
print_every_n   = 20;      % print a progress line every N received lines (when print_all_lines=false)
print_every_s   = 2;       % also print a heartbeat every N seconds

%% ====== MAKE OUTPUT DIR FIRST (so outDir always exists) ======
runTime = datetime("now");
dateStr = char(datetime(runTime,"Format","yyyy-MM-dd"));
timeStr = char(datetime(runTime,"Format","HHmmss"));

cmd = upper(string(cmd));
if cmd == "DZ"
    kind = "Deadzone";
elseif cmd == "RS"
    kind = "RatioScan";
else
    error("cmd must be 'DZ' or 'RS'.");
end

outDir = fullfile(baseOutDir, kind, dateStr, timeStr);
if ~exist(outDir, "dir"), mkdir(outDir); end
fprintf("Output dir: %s\n", outDir);

fid = fopen(fullfile(outDir, "meta.txt"), "w");
fprintf(fid, "cmd=%s\n", cmd);
fprintf(fid, "port=%s, baud=%d\n", port, baud);
fprintf(fid, "timestamp=%s\n", char(runTime));
fclose(fid);

%% ====== OPEN SERIAL, FLUSH, SEND COMMAND ======
s = serialport(port, baud);
configureTerminator(s, "LF");
s.Timeout = 1;

% Flush buffers (compatible across versions)
try
    flush(s);
catch
    try
        flush(s,"input");
        flush(s,"output");
    catch
        % If flush isn't available, ignore.
    end
end
pause(0.2);

disp("Sending command: " + cmd);
writeline(s, cmd);

%% ====== RECEIVE LOOP WITH PROGRESS PRINTING ======
disp("Collecting... (waiting for # done)");
rawLines = strings(0,1);

t0 = tic;
gotDone = false;

lineCount = 0;
lastHeartbeat = tic;

while toc(t0) < timeout_s
    if s.NumBytesAvailable == 0
        pause(0.01);
        continue;
    end

    line = strtrim(readline(s));
    rawLines(end+1,1) = line; %#ok<AGROW>
    lineCount = lineCount + 1;

    % ---- printing ----
    if print_all_lines
        disp(line);
    else
        % Always print control lines
        if startsWith(line, "#")
            disp(line);
        end

        % Print progress every N lines using the latest data line
        if mod(lineCount, print_every_n) == 0
            if startsWith(line, "DATA,")
                p = split(line, ",");
                if numel(p) == 7
                    fprintf("...lines=%d  dir=%s  pwm=%s  rep=%s  ratio=%s\n", ...
                        lineCount, p(2), p(3), p(4), p(7));
                else
                    fprintf("...lines=%d\n", lineCount);
                end
            elseif startsWith(line, "DZ,")
                p = split(line, ",");
                if numel(p) == 6
                    fprintf("...lines=%d  wheel=%s  pwm=%s  rep=%s  moving=%s\n", ...
                        lineCount, p(2), p(3), p(4), p(6));
                else
                    fprintf("...lines=%d\n", lineCount);
                end
            else
                fprintf("...lines=%d\n", lineCount);
            end
        end

        % Heartbeat print every few seconds
        if toc(lastHeartbeat) > print_every_s
            fprintf("...running  elapsed=%.1fs  total_lines=%d\n", toc(t0), lineCount);
            lastHeartbeat = tic;
        end
    end

    % ---- exact done condition ----
    if strcmpi(line, "# done")
        gotDone = true;
        break;
    end
end

clear s;  % close serial

% Save raw lines text no matter what
saveRawText(fullfile(outDir, "raw_lines.txt"), rawLines);

if ~gotDone
    warning("Timeout before receiving '# done'. Still saving what we got.");
end

%% ====== PARSE + SAVE CSV + PLOTS ======
if cmd == "DZ"
    % Parse Deadzone rows: DZ,wheel,pwm,repeat,cps,is_moving
    rows = {};
    for i = 1:numel(rawLines)
        line = rawLines(i);
        if ~startsWith(line, "DZ,"), continue; end
        p = split(line, ",");
        if numel(p) ~= 6, continue; end
        rows(end+1,:) = {string(p(2)), str2double(p(3)), str2double(p(4)), str2double(p(5)), str2double(p(6))}; %#ok<AGROW>
    end
    if isempty(rows), error("No DZ data parsed. Check Arduino output."); end

    DZ = cell2table(rows, "VariableNames", ["wheel","pwm","rep","cps","is_moving"]);
    writetable(DZ, fullfile(outDir, "deadzone_raw.csv"));

    DZsum = groupsummary(DZ, ["wheel","pwm"], ["mean","std"], ["cps","is_moving"]);
    writetable(DZsum, fullfile(outDir, "deadzone_summary.csv"));

    wheels = unique(DZsum.wheel);

    % Estimate deadzone and save
    fid = fopen(fullfile(outDir, "deadzone_estimate.txt"), "w");
    fprintf(fid, "Deadzone estimate: first pwm with mean_is_moving >= 0.67\n");
    for w = 1:numel(wheels)
        gw = DZsum(DZsum.wheel==wheels(w), :);
        gw = sortrows(gw, "pwm");
        idx = find(gw.mean_is_moving >= 0.67, 1, "first");
        if ~isempty(idx)
            fprintf(fid, "wheel %s : pwm >= %d\n", wheels(w), gw.pwm(idx));
        else
            fprintf(fid, "wheel %s : not found in scanned range\n", wheels(w));
        end
    end
    fclose(fid);

    % Plot 1: mean cps vs pwm
    fig1 = figure("Name","Deadzone_mean_cps","NumberTitle","off");
    hold on; grid on;
    for w = 1:numel(wheels)
        gw = DZsum(DZsum.wheel==wheels(w), :);
        gw = sortrows(gw, "pwm");
        plot(gw.pwm, gw.mean_cps, "-o", "DisplayName", "wheel "+wheels(w));
    end
    xlabel("PWM"); ylabel("mean cps");
    title("Deadzone (no-load): mean cps vs PWM");
    legend("Location","best");
    savefig(fig1, fullfile(outDir, "deadzone_mean_cps.fig"));

    % Plot 2: moving probability vs pwm
    fig2 = figure("Name","Deadzone_moving_prob","NumberTitle","off");
    hold on; grid on;
    for w = 1:numel(wheels)
        gw = DZsum(DZsum.wheel==wheels(w), :);
        gw = sortrows(gw, "pwm");
        plot(gw.pwm, gw.mean_is_moving, "-o", "DisplayName", "wheel "+wheels(w));
    end
    xlabel("PWM"); ylabel("mean is\_moving (0..1)");
    title("Deadzone (no-load): moving probability vs PWM");
    ylim([0 1.05]);
    legend("Location","best");
    savefig(fig2, fullfile(outDir, "deadzone_moving_prob.fig"));

elseif cmd == "RS"
    % Parse RatioScan rows: DATA,dir,pwm,repeat,cpsL,cpsR,ratio
    rows = {};
    for i = 1:numel(rawLines)
        line = rawLines(i);
        if ~startsWith(line, "DATA,"), continue; end
        p = split(line, ",");
        if numel(p) ~= 7, continue; end
        rows(end+1,:) = {str2double(p(2)), str2double(p(3)), str2double(p(4)), ...
                         str2double(p(5)), str2double(p(6)), str2double(p(7))}; %#ok<AGROW>
    end
    if isempty(rows), error("No DATA parsed. Check Arduino output."); end

    T = cell2table(rows, "VariableNames", ["dir","pwm","rep","cpsL","cpsR","ratio"]);
    writetable(T, fullfile(outDir, "ratio_raw.csv"));

    S = groupsummary(T, ["dir","pwm"], ["mean","std"], ["cpsL","cpsR","ratio"]);
    writetable(S, fullfile(outDir, "ratio_summary.csv"));

    dirs = unique(S.dir);

    % Plot 1: k(PWM)=mean ratio vs pwm
    fig3 = figure("Name","Ratio_k_curve","NumberTitle","off");
    hold on; grid on;
    for d = 1:numel(dirs)
        sd = S(S.dir==dirs(d), :);
        sd = sortrows(sd, "pwm");
        plot(sd.pwm, sd.mean_ratio, "-o", "DisplayName", "dir "+string(dirs(d)));
    end
    xlabel("PWM"); ylabel("mean(cpsL/cpsR)");
    title("k(PWM)=cpsL/cpsR (no-load)");
    legend("Location","best");
    savefig(fig3, fullfile(outDir, "ratio_k_curve.fig"));

    % Plot 2: mean cpsL/cpsR vs pwm
    fig4 = figure("Name","Ratio_cps_curve","NumberTitle","off");
    hold on; grid on;
    for d = 1:numel(dirs)
        sd = S(S.dir==dirs(d), :);
        sd = sortrows(sd, "pwm");
        plot(sd.pwm, sd.mean_cpsL, "-o", "DisplayName", "cpsL dir "+string(dirs(d)));
        plot(sd.pwm, sd.mean_cpsR, "-x", "DisplayName", "cpsR dir "+string(dirs(d)));
    end
    xlabel("PWM"); ylabel("mean cps");
    title("Wheel speed vs PWM (no-load)");
    legend("Location","best");
    savefig(fig4, fullfile(outDir, "ratio_cps_curve.fig"));

    % Write suggested universal k (optional)
    pos = S(S.dir==1, :);
    pos = sortrows(pos, "pwm");
    good = pos.mean_cpsR > 200; % avoid deadzone/noisy points
    if any(good)
        k_med = median(pos.mean_ratio(good));
        k_ls  = sum(pos.mean_cpsL(good).*pos.mean_cpsR(good)) / sum(pos.mean_cpsR(good).^2);
    else
        k_med = NaN; k_ls = NaN;
    end

    fid = fopen(fullfile(outDir, "k_suggestion.txt"), "w");
    fprintf(fid, "Suggested universal k (forward only)\n");
    fprintf(fid, "Filter: mean_cpsR > 200\n");
    fprintf(fid, "k_median = %.6f\n", k_med);
    fprintf(fid, "k_least_squares = %.6f\n", k_ls);
    fclose(fid);
end

disp("Done. Results saved under:");
disp(outDir);

%% ====== LOCAL HELPERS ======
function saveRawText(filepath, lines)
    % Compatible with MATLAB versions without writelines()
    if exist("writelines","file") == 2
        writelines(lines, filepath);
    else
        fid = fopen(filepath, "w");
        for i = 1:numel(lines)
            fprintf(fid, "%s\n", lines(i));
        end
        fclose(fid);
    end
end
