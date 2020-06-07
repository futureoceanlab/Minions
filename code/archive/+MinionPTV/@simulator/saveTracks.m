function [this] =saveTracks(this)
% Author: Junsu Jang (junsu.jang94@gmail.com)
    % Remove tracks that did not appear at all
    this.tracks(this.tracks.age == 0, :) = [];
    save(this.trackFile, 'this.tracks');
    fprintf("Successfully saved the tracks\n");
end