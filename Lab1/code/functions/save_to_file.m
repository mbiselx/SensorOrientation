function out = save_to_file(filename, header, data)

    fid = fopen(filename, 'w');

    fprintf(fid, header);

    cols = columns(data);
    for r = 1:rows(data)
        for c = 1:cols
            fprintf(fid, "%.8f", data(r, c));
            if (c < cols)
                fprintf(fid, "\t");
            else
                fprintf(fid, "\n");
            end
        end
    end

    fclose(fid);

end
