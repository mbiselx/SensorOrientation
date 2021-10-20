function save_to_file(filename, header, data, separator)

    fid = fopen(filename, 'w');

    fprintf("saving to file %s\n", filename);
    fprintf(fid, header);

    cols = size(data,2);
    for r = 1:size(data,1)
        for c = 1:cols
            fprintf(fid, "%.8f", data(r, c));
            if (c < cols)
                fprintf(fid, "%c\t", separator);
            else
                fprintf(fid, "\n");
            end
        end
    end

    fclose(fid);

end
